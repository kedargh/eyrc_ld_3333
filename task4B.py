# import the necessary packages

from imutils import contours
from skimage import measure
import numpy as np
import argparse
import imutils
import cv2

#----------------------------------------------------------------------------------------------------------

# load the image
parser = argparse.ArgumentParser()
parser.add_argument('--image')
args = parser.parse_args()
print(args)
img = cv2.imread(args.image)

#----------------------------------------------------------------------------------------------------------

#implementation logic updated for stage 2
erosionimg = cv2.erode(img, np.ones((7,7)), iterations=1)
grayimg = cv2.cvtColor(erosionimg, cv2.COLOR_BGR2GRAY)
_, thresholdimg = cv2.threshold(grayimg, 180, 255, cv2.THRESH_BINARY)
dilationimg = cv2.dilate(thresholdimg, np.ones((7,7)), iterations=1)

#--------------------------------------------------------------------------------------------------------------

# perform a connected component analysis on the thresholded image, then initialize a mask to store only the "large" components

nos_Labels, label_id, val, centroid = cv2.connectedComponentsWithStats(dilationimg, 4, cv2.CV_32S)

#---------------------------------------------------------------------------------------------------------------

# find the contours in the mask, then sort them from left to right

contours = cv2.findContours(dilationimg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
contours = imutils.grab_contours(contours)
contours = sorted(contours, key=lambda c: cv2.boundingRect(c)[0])

#----------------------------------------------------------------------------------------------

# Initialize lists to store centroid coordinates and area

centroid_list = []
area_list = []

#----------------------------------------------------------------------------------------------

# Loop over the contours
for cont in contours:
    # Calculate the area of the contour
    area = cv2.contourArea(cont)
    # Calculate the centroid coordinates
    M = cv2.moments(cont)
    if M['m00'] != 0:
        cx = float(M['m10'] / M['m00'])
        cy = float(M['m01'] / M['m00'])
    else:
        # Handle the case when the area (m00) is zero to avoid division by zero
        cx, cy = 0, 0

    # Append centroid coordinates and area to the respective lists
    area_list.append(area)
    centroid_list.append((cx , cy))

# Draw the bright spot on the image
cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
#------------------------------------------------------------------------------------------------

# Save the output image as a PNG file
cv2.imwrite("led_detection_results.png", img)

# Open a text file for writing
with open("led_detection_results.txt", "w") as file:
    # Write the number of LEDs detected to the file
    file.write(f"No. of LEDs detected: {len(centroid_list)}\n")

    # Loop over the contours
    for i in range (0,len(centroid_list)):
        # Write centroid coordinates and area for each LED to the file
        file.write(f"Centroid #{i + 1}: {centroid_list[i]}\nArea #{i + 1}: {area_list[i]}\n")
# Close the text file
file.close()
