import numpy as np

def image_to_bgr(msg):
    channels=3
    if msg.encoding.lower() in ('rgb8','rgb16'):
        arr=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3)
        return arr[:,:,::-1].copy()
    if msg.encoding.lower() in ('bgr8','bgr16'):
        return np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width*3].reshape(msg.height,msg.width,3).copy()
    raise ValueError(f'unsupported image encoding: {msg.encoding}')
