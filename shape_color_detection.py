import cv2
import numpy as np
import matplotlib.pyplot as plt
def check_boundry(img,range,lower_bound,upper_bound):
    frist = img[:,:,0]
    sec = img[:,:,1]
    thierd = img[:,:,2]
    y = range[0,0]+5
    x = range[0,1]+5
    return frist[x,y] >= lower_bound[0] and frist[x,y] <= upper_bound[0] and sec[x,y] >= lower_bound[1] and sec[x,y] <= upper_bound[1] and thierd[x,y] >= lower_bound[2] and thierd[x,y] <= upper_bound[2]

def color_detect(img,range):
    img = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    color= "unknown"

    #for red
    lower_bound_r = np.array([0, 100, 50], dtype=np.uint8)
    upper_bound_r = np.array([10, 255, 255], dtype=np.uint8)

    #for blue
    lower_bound_b = np.array([100, 100, 100], dtype=np.uint8)
    upper_bound_b = np.array([140, 255, 255], dtype=np.uint8)


    #for green
    lower_bound_g = np.array([44, 150, 50], dtype=np.uint8)
    upper_bound_g = np.array([65, 255, 255], dtype=np.uint8)


    #for yellow
    lower_bound_y = np.array([20, 150, 50], dtype=np.uint8)
    upper_bound_y = np.array([30, 255, 255], dtype=np.uint8)

    if  check_boundry(img,range,lower_bound_r,upper_bound_r):
        color = "red"
    elif check_boundry(img,range,lower_bound_b,upper_bound_b) :
        color = "blue"
    elif check_boundry(img,range,lower_bound_g,upper_bound_g):
        color = "green"
    elif check_boundry(img,range,lower_bound_y,upper_bound_y) :
        color = "yellow"
    return color

image = cv2.imread("test.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blurred = cv2.GaussianBlur(gray, (3, 3), 0)



edges = cv2.Canny(blurred, 10, 50)


contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

for contour in contours:
    
    epsilon = 0.008 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    x, y, w, h = cv2.boundingRect(approx)
    
    if len(approx) == 3:
        shape = "Triangle"
    elif len(approx) == 4:
        aspect_ratio = w / float(h)
        if 0.95 <= aspect_ratio <= 1.05:
            shape = "Square"
        else:
            shape = "Rectangle"
    elif len(approx) > 4:
        shape = "Circle"
    else:
        shape = "Unknown"
    color = color_detect(image,approx[0])

    cv2.putText(image, color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.putText(image, shape, (x-5, y-15 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
    cv2.drawContours(image, [approx], -1, (0, 0, 0), 5)

plt.figure(figsize=(12, 6))
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))  
plt.axis('off')
plt.show()