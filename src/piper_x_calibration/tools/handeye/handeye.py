#!/usr/bin/env python3
"""Manual eye-in-hand ChArUco calibration for PiPER-X."""
import argparse
import json
from pathlib import Path
import time

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def read_yaml(path):
    return yaml.safe_load(Path(path).read_text())


def board_from(spec):
    ar = cv2.aruco
    sx, sy = spec['squares_x'], spec['squares_y']
    square, marker = spec['square_m'], spec['marker_m']
    if min(sx, sy) < 3 or not 0 < marker < square:
        raise ValueError('Need at least 3 × 3 squares and 0 < marker_m < square_m')
    dictionary = ar.getPredefinedDictionary(getattr(ar, spec['dictionary']))
    args = (sx, sy), square, marker, dictionary
    if hasattr(ar, 'CharucoBoard'):
        board = ar.CharucoBoard(*args)
        board.setLegacyPattern(bool(spec.get('legacy_pattern', True)))
    else:
        if not spec.get('legacy_pattern', True):
            raise ValueError('This OpenCV only supports the legacy board pattern')
        board = ar.CharucoBoard_create(sx, sy, square, marker, dictionary)
    return board, dictionary


def transform(rotation, translation):
    m = np.eye(4)
    m[:3, :3] = rotation
    m[:3, 3] = np.asarray(translation).reshape(3)
    return m


def estimate_board(gray, k, distortion, board, dictionary):
    ar = cv2.aruco
    corners, ids, _ = ar.detectMarkers(gray, dictionary)
    if ids is None:
        raise ValueError('No ArUco markers found')
    count, pixels, ids = ar.interpolateCornersCharuco(corners, ids, gray, board,
                                                       cameraMatrix=k, distCoeffs=distortion)
    if not count or count < 15:
        raise ValueError(f'Only {count} ChArUco corners, need at least 15')
    points = board.getChessboardCorners() if hasattr(board, 'getChessboardCorners') else board.chessboardCorners
    points = points[ids.ravel()]
    ok, r, t = cv2.solvePnP(points, pixels, k, distortion)
    if not ok or t[2, 0] <= 0:
        raise ValueError('Board pose estimation failed')
    projected = cv2.projectPoints(points, r, t, k, distortion)[0]
    rms = float(np.sqrt(np.mean(np.sum((projected-pixels)**2, axis=-1))))
    return transform(cv2.Rodrigues(r)[0], t), rms, int(count)


def capture(args, spec, profile):
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import CameraInfo, Image
    from tf2_ros import Buffer, TransformListener, TransformException

    board, dictionary = board_from(spec)
    rclpy.init()
    node = rclpy.create_node('piper_handeye_capture', parameter_overrides=[
        Parameter('use_sim_time', value=profile['use_sim_time'])])
    tf = Buffer(node=node)
    listener = TransformListener(tf, node)
    latest = {}
    node.create_subscription(Image, profile['image_topic'],
                             lambda msg: latest.update(image=msg), qos_profile_sensor_data)
    node.create_subscription(CameraInfo, profile['camera_info_topic'],
                             lambda msg: latest.update(info=msg), qos_profile_sensor_data)
    path = Path(args.samples)
    meta = {'board': spec, 'profile': profile}
    if path.exists() and path.stat().st_size:
        data = json.loads(path.read_text())
        if data['metadata'] != meta:
            raise ValueError('Existing samples use a different board or environment')
    else:
        data = {'metadata': meta, 'samples': []}
    print('Move the arm, let it settle, then press Enter. Type q to finish.', flush=True)
    try:
        while input('Capture [Enter/q]: ').strip().lower() != 'q':
            latest.clear()
            accepted = False
            deadline = time.monotonic() + 5
            error = 'Waiting for fresh image and CameraInfo'
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.1)
                if not {'image', 'info'} <= latest.keys():
                    continue
                image, info = latest['image'], latest['info']
                stamp = Time.from_msg(image.header.stamp)
                if not stamp.nanoseconds or (image.width,image.height) != (info.width,info.height):
                    error = 'Invalid image timestamp or CameraInfo dimensions'
                    continue
                if image.header.frame_id != profile['optical_frame'] or info.header.frame_id != image.header.frame_id:
                    error = 'Optical frame differs from profile or CameraInfo'
                    continue
                try:
                    msg = tf.lookup_transform(profile['base_frame'],profile['wrist_frame'],stamp).transform
                except TransformException as ex:
                    error = str(ex)
                    continue
                try:
                    encoding = image.encoding.lower()
                    if encoding not in ('rgb8','bgr8','mono8','rgba8','bgra8'):
                        raise ValueError(f'Unsupported encoding {encoding}')
                    channels = {'mono8':1,'rgb8':3,'bgr8':3,'rgba8':4,'bgra8':4}[encoding]
                    raw = np.frombuffer(image.data,np.uint8).reshape(image.height,image.step)
                    raw = raw[:,:image.width*channels].reshape(image.height,image.width,channels)
                    gray = raw[:,:,0] if channels == 1 else cv2.cvtColor(raw,getattr(cv2,'COLOR_'+encoding[:-1].upper()+'2GRAY'))
                    if info.distortion_model not in ('','plumb_bob','rational_polynomial'):
                        raise ValueError(f'Unsupported distortion model {info.distortion_model}')
                    k=np.asarray(info.k).reshape(3,3)
                    if min(k[0,0],k[1,1]) <= 0:
                        raise ValueError('CameraInfo has no valid intrinsics')
                    observed, rms, count = estimate_board(gray,k,np.asarray(info.d),board,dictionary)
                    if rms > profile.get('max_reprojection_px', 1.0):
                        raise ValueError(f'Reprojection error {rms:.2f} px')
                    if any(sample['stamp_ns']==stamp.nanoseconds for sample in data['samples']):
                        raise ValueError('Duplicate image timestamp')
                except (ValueError, cv2.error) as ex:
                    error = str(ex)
                    break
                q,p=msg.rotation,msg.translation
                wrist=transform(Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix(),[p.x,p.y,p.z])
                data['samples'].append(dict(stamp_ns=stamp.nanoseconds, base_from_wrist=wrist.tolist(),
                                            camera_from_board=observed.tolist(), corners=count, reprojection_px=rms))
                path.parent.mkdir(parents=True,exist_ok=True)
                tmp=path.with_suffix(path.suffix+'.tmp')
                tmp.write_text(json.dumps(data,indent=2)+'\n')
                tmp.replace(path)
                accepted = True
                print(f'Accepted {len(data["samples"])}: {count} corners, {rms:.2f} px',flush=True)
                break
            else:
                print(f'Not captured: {error}',flush=True)
                continue
            if not accepted:
                print(f'Not captured: {error}',flush=True)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def arrays(samples):
    return (np.asarray([s['base_from_wrist'] for s in samples]),
            np.asarray([s['camera_from_board'] for s in samples]))


def solve(samples):
    wrist, board = arrays(samples)
    relative = Rotation.from_matrix(wrist[0,:3,:3].T @ wrist[:,:3,:3]).as_rotvec()
    if np.linalg.svd(relative,compute_uv=False)[1] < np.deg2rad(10):
        raise ValueError('Insufficient rotation about independent axes')
    r,t = cv2.calibrateHandEye(wrist[:,:3,:3],wrist[:,:3,3],board[:,:3,:3],board[:,:3,3],
                              method=cv2.CALIB_HAND_EYE_PARK)
    if not np.isfinite(r).all() or not np.isfinite(t).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or np.linalg.det(r)<0:
        raise ValueError('Hand-eye solver returned an invalid transform')
    return transform(r,t)


def validation(samples, wrist_from_camera, training):
    a,b=arrays(training)
    reference_poses=a@wrist_from_camera@b
    ref_t=reference_poses[:,:3,3].mean(axis=0)
    ref_r=Rotation.from_matrix(reference_poses[:,:3,:3]).mean()
    a,b=arrays(samples)
    poses=a@wrist_from_camera@b
    translation=np.linalg.norm(poses[:,:3,3]-ref_t, axis=1)*1000
    angle=(ref_r.inv()*Rotation.from_matrix(poses[:,:3,:3])).magnitude()*180/np.pi
    return dict(samples=len(samples), translation_rms_mm=float(np.sqrt(np.mean(translation**2))),
                translation_max_mm=float(translation.max()), rotation_rms_deg=float(np.sqrt(np.mean(angle**2))),
                rotation_max_deg=float(angle.max()))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='command',required=True)
    for name in ('capture','board'):
        p=sub.add_parser(name)
        p.add_argument('--board', default=str(Path(__file__).with_name('board.yaml')))
        if name=='capture':
            p.add_argument('--profile',required=True)
            p.add_argument('--samples',required=True)
        else:
            p.add_argument('--output',required=True)
    p=sub.add_parser('solve')
    p.add_argument('--samples',required=True)
    p.add_argument('--output',required=True)
    p.add_argument('--holdout',type=int,default=4)
    args=parser.parse_args()
    if args.command=='capture':
        capture(args,read_yaml(args.board),read_yaml(args.profile))
    elif args.command=='board':
        spec=read_yaml(args.board)
        board,_=board_from(spec)
        size=(spec['squares_x']*160,spec['squares_y']*160)
        image=board.generateImage(size) if hasattr(board,'generateImage') else board.draw(size)
        image=cv2.copyMakeBorder(image,160,160,160,160,cv2.BORDER_CONSTANT,value=255)
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        if not cv2.imwrite(args.output,image):
            raise OSError(f'Cannot write {args.output}')
        Path(args.output).with_suffix('.json').write_text(json.dumps(spec)+'\n')
    else:
        data=json.loads(Path(args.samples).read_text())
        samples=data['samples']
        if args.holdout<3 or len(samples)-args.holdout<8:
            parser.error('Need at least 8 training and 3 held-out samples')
        train,test=samples[:-args.holdout],samples[-args.holdout:]
        model=solve(train)
        prof=data['metadata']['profile']
        result=dict(parent_frame=prof['wrist_frame'],child_frame=prof['optical_frame'],
                    transform_convention='parent_T_child',
                    translation_m=model[:3,3].tolist(),
                    quaternion_xyzw=Rotation.from_matrix(model[:3,:3]).as_quat().tolist(),
                    training_samples=len(train),validation=validation(test,model,train))
        output=Path(args.output)
        output.parent.mkdir(parents=True,exist_ok=True)
        with output.open('x') as file:
            yaml.safe_dump(result,file,sort_keys=False)
        print(yaml.safe_dump(result,sort_keys=False),end='')


if __name__=='__main__':
    main()
