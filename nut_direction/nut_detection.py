from pickle import FALSE, TRUE
import cv2
import numpy as np

"""# 1. Load image"""
image_1 = cv2.imread('nut.png')
down_width = 1500
down_height = 1000
down_points = (down_width, down_height)
image_1 = cv2.resize(image_1, down_points, interpolation= cv2.INTER_LINEAR)
image = image_1.copy()


"""# 2. Detect nut"""

lower_bound = np.array([40, 60, 0])
upper_bound = np.array([180, 255, 255])
image_1_HSV = cv2.cvtColor(image_1, cv2.COLOR_BGR2HSV)
image_1_masked = cv2.inRange(image_1_HSV, lower_bound, upper_bound)
# cv2.imshow("image_1_masked", image_1_masked)
# cv2.waitKey()
# cv2.destroyAllWindows()

image_1_dil = cv2.dilate(image_1_masked, np.ones((15, 15), np.uint8))
# image_1_ero = cv2.erode(image_1_dil, np.ones((8, 8), np.uint8))

# cv2.imshow("image_1_dil", image_1_dil)
# cv2.waitKey()
# cv2.destroyAllWindows()
# cv2.imshow("image_1_ero", image_1_ero)
# cv2.waitKey()
# cv2.destroyAllWindows()

conts, hierarchy = cv2.findContours(image_1_dil, cv2.RETR_LIST ,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image_1, conts, -1, (255, 0, 0), 2)

centers = []
for cont in conts:
    M = cv2.moments(cont)
    x0 = int(M['m10']/M['m00'])
    y0 = int(M['m01']/M['m00'])
    centers.append((x0, y0))

for center in centers:
    cv2.circle(image, center, 3, (255, 0, 0), 3)

# Fin the furthest point of the contour (from its center of mass)
max_dist = 0
max_point = None

for cont in conts:
    for point in cont:
        dist = np.sqrt((x0 - point[0][0])**2 + (y0 - point[0][1])**2)
        if dist > max_dist:
            max_dist = dist
            max_point = point[0]

cv2.circle(image, max_point, 2, (255, 0, 0), 3)
print('center: ' + str(center))
print('max_point: ' + str(max_point))



# Calculating the tool angle
offset_angle_of_camera = 20
tool_angle = 0


if (max_point[0]>center[0] and max_point[1]<center[1]) or (max_point[0]<center[0] and max_point[1]>center[1]):
    tool_angle = -(90 - np.degrees(np.arcsin((center[1] - max_point[1])/(max_dist))) + 18.5)
elif (max_point[0]<center[0] and max_point[1]<center[1]) or (max_point[0]>center[0] and max_point[1]>center[1]):
    tool_angle = 90 - np.degrees(np.arcsin((center[1] - max_point[1])/(max_dist))) - 18.5
elif point[0]==center[0]:
    tool_angle = -18.5
elif point[1]==center[1]:
    tool_angle = 90-18.5

print('tool angle: ' + str(tool_angle))
# Display
cv2.imshow("image", image)
cv2.waitKey()
cv2.destroyAllWindows()