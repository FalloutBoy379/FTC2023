import cv2 
import numpy as np  

cap = cv2.VideoCapture(1)

while(1):        
    _, frame = cap.read()  
    original  = frame.copy();
    blank = np.zeros(frame.shape, dtype=np.uint8)
    blur = cv2.GaussianBlur(frame, (7,7), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    lower_val = np.array([0,0,0])
    upper_val = np.array([179,93,97])
    mask = cv2.inRange(hsv, lower_val, upper_val) 
    
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7,7))
    opening = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
    close = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel, iterations=2)
    
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = cnts[0] if len(cnts) == 2 else cnts[1]
    for c in cnts:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.04 * peri, True)
        area = cv2.contourArea(c)
        if len(approx) > 3 and area > 1000:
            cv2.drawContours(frame, [c], -1, (36,255,12), -1)
            cv2.drawContours(blank, [c], -1, (255,255,255), -1)



    blank = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
    result = cv2.bitwise_and(original,original,mask=blank)
    result[blank==0] = (255,255,255)
    
    
    cv2.imshow('mask', mask)
    cv2.imshow('opening', opening)
    cv2.imshow('close', close)
    cv2.imshow('result', result)
    cv2.imshow('image', frame)
    k = cv2.waitKey(5) & 0xFF
    if k == 27: 
        break

cv2.destroyAllWindows() 
cap.release()