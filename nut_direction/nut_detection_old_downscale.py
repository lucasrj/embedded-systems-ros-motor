from pickle import FALSE, TRUE
import cv2
import numpy as np

"""# 1. Load image"""
image_1 = cv2.imread('num_710.png')
down_width = 640
down_height = 480
down_points = (down_width, down_height)
image_1 = cv2.resize(image_1, down_points, interpolation= cv2.INTER_LINEAR)
image = image_1.copy()


"""# 2. Detect nut"""
lower_bound = np.array([15, 0, 0])
upper_bound = np.array([255, 100, 255])
image_1_HSV = cv2.cvtColor(image_1, cv2.COLOR_BGR2HSV)
image_1_masked = cv2.inRange(image_1_HSV, lower_bound, upper_bound)
# cv2.imshow("image_1_masked", image_1_masked)
# cv2.waitKey()
# cv2.destroyAllWindows()

image_1_dil = cv2.dilate(image_1_masked, np.ones((15, 15), np.uint8))
image_1_ero = cv2.erode(image_1_dil, np.ones((10, 10), np.uint8))
image_1 = cv2.bitwise_not(image_1_ero)

# cv2.imshow("image_1_dil", image_1_dil)
# cv2.waitKey()
# cv2.destroyAllWindows()
# cv2.imshow("image_1_ero", image_1_ero)
# cv2.waitKey()
# cv2.destroyAllWindows()
# cv2.imshow("image_1", image_1)
# cv2.waitKey()
# cv2.destroyAllWindows()

conts, hierarchy = cv2.findContours(image_1, cv2.RETR_EXTERNAL ,cv2.CHAIN_APPROX_SIMPLE)
cv2.drawContours(image_1, conts, -1, (8, 81, 100), 2)
cv2.imshow("image_1_contours", image_1)
cv2.waitKey()
cv2.destroyAllWindows()

centers = []
for cont in conts:
    M = cv2.moments(cont)
    x0 = int(M['m10']/M['m00'])
    y0 = int(M['m01']/M['m00'])
    centers.append((x0, y0))

for center in centers:
    cv2.circle(image, center, 3, (5, 92, 94), 3)
    # Draw line where vertex should be
    a = 100 * np.tan(np.radians(18.5))
    print(a)
    cv2.line(image, center, (center[0]-int(a), center[1]-100), (5, 92, 94), 1)

# Find the furthest point of the contour (from its center of mass)
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
offset_angle_of_camera = 18.5
tool_angle = 0
    
if (center[0]<max_point[0] and center[1]>max_point[1]):
    print('I')
    tool_angle = -np.degrees(np.arctan((max_point[0]-center[0])/(center[1]-max_point[1]))) - offset_angle_of_camera
elif (center[0]<max_point[0] and center[1]<max_point[1]):
    print('IV')
    tool_angle = 90 - offset_angle_of_camera - np.degrees(np.arctan((max_point[1]-center[1])/(max_point[0]-center[0])))
elif (center[0]>max_point[0] and center[1]<max_point[1]):
    print('III')
    tool_angle = -np.degrees(np.arctan((center[0]-max_point[0])/(max_point[1]-center[1]))) - offset_angle_of_camera
elif (center[0]>max_point[0] and center[1]>max_point[1]):
    print('II')
    tool_angle = 90 - offset_angle_of_camera - np.degrees(np.arctan((center[1]-max_point[1])/(center[0]-max_point[0])))
elif point[0]==center[0]:
    tool_angle = -18.5
elif point[1]==center[1]:
    tool_angle = 90-18.5
    
print('tool angle: ' + str(tool_angle))
# Display
cv2.imshow("image", image)
cv2.waitKey()
cv2.destroyAllWindows()