import cv2
import numpy as np

def detect_color(image_bgr, lower=(90,70,40), upper=(140,255,255), min_area=100):
    hsv=cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.asarray(lower,dtype=np.uint8),np.asarray(upper,dtype=np.uint8))
    kernel=np.ones((5,5),np.uint8)
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
    contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    valid=[c for c in contours if cv2.contourArea(c)>=min_area]
    if not valid: return None,mask
    c=max(valid,key=cv2.contourArea); m=cv2.moments(c)
    if m['m00']==0: return None,mask
    x=float(m['m10']/m['m00']); y=float(m['m01']/m['m00']); box=cv2.boundingRect(c)
    return {'center_uv':(x,y),'box_xywh':box,'area_px':float(cv2.contourArea(c))},mask
