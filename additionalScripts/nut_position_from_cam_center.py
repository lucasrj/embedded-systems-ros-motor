from pickle import FALSE, TRUE
import cv2
import numpy as np

"""# Load image"""
image_1 = cv2.imread('num_710.png')
image = image_1.copy()
c = (int((image.shape[1])/2), int((image.shape[0])/2))

"""# Detect nut"""
lower_bound = np.array([15, 0, 0])
upper_bound = np.array([255, 100, 255])
image_1_HSV = cv2.cvtColor(image_1, cv2.COLOR_BGR2HSV)
image_1_masked = cv2.inRange(image_1_HSV, lower_bound, upper_bound)

image_1_dil = cv2.dilate(image_1_masked, np.ones((15, 15), np.uint8))
image_1_ero = cv2.erode(image_1_dil, np.ones((10, 10), np.uint8))
image_1 = cv2.bitwise_not(image_1_ero)

conts, hierarchy = cv2.findContours(image_1, cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image, conts, -1, (8, 81, 100), 2)
# cv2.imshow("image", image)
# cv2.waitKey()
# cv2.destroyAllWindows()

centers = []
for cont in conts:
    M = cv2.moments(cont)
    x0 = int(M['m10']/M['m00'])
    y0 = int(M['m01']/M['m00'])
    centers.append((x0, y0))
    
"""# Calculate position"""
for center in centers:
    cv2.circle(image, center, 3, (5, 92, 94), 3)
    cv2.circle(image, c, 3, (0, 82, 88), 3)
    nutpos = (center[0] - c[0], center[1] - c[1])
    print(nutpos)

cv2.imshow("image", image)
cv2.waitKey()
cv2.destroyAllWindows()