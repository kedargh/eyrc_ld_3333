# import the necessary packages

import numpy as np
import argparse
import imutils
import cv2

#----------------------------------------------------------------------------------------------------------

# load the image
parser = argparse.ArgumentParser()
parser.add_argument('--image')
args = parser.parse_args()
img = cv2.imread(args.image)
txt_filename = (args.image).replace(".png",".txt")
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
centroids = []   #[alien type , no of leds and centroid coordinates]
def centroid_calculator(centroid_list):
    alien =  (len(centroid_list))
    sumofx = sum(xcoor for xcoor , _ in centroid_list)
    sumofy = sum(ycoor for _ , ycoor in centroid_list)
    centroidx = sumofx/alien
    centroidy = sumofy/alien
    
    if alien == 2 :
        file.write(f"Organism Type: alien_a\n" + f"Centroid: {(centroidx,centroidy)}\n\n")
        centroids.append(['alien_a' , 2 ,centroidx,centroidy])
    elif alien == 3 :
        file.write(f"Organism Type: alien_b\n" + f"Centroid: {(centroidx,centroidy)}\n\n")
        centroids.append(['alien_b' , 3 ,centroidx,centroidy])
    elif alien == 4 :
        file.write(f"Organism Type: alien_c\n" + f"Centroid: {(centroidx,centroidy)}\n\n")
        centroids.append(['alien_c' , 4 ,centroidx,centroidy])
    elif alien == 5 :
        file.write(f"Organism Type: alien_d\n" + f"Centroid: {(centroidx,centroidy)}\n\n")
        centroids.append(['alien_d' , 5 ,centroidx,centroidy])
    elif alien >= 6 :
        centroid_1 = []  # x < 256, y < 256
        centroid_2 = []  # x < 256, y > 256
        centroid_3 = []  # x > 256, y < 256
        centroid_4 = []  # x > 256, y > 256

        for x, y in centroid_list:
            if x < 256 and y < 256:
                centroid_1.append((x, y))
            elif x < 256 and y > 256:
                centroid_2.append((x, y))
            elif x > 256 and y < 256:
                centroid_3.append((x, y))
            else:
                centroid_4.append((x, y))

        if len(centroid_1) != 0:
            centroid_calculator(centroid_1)

        if len(centroid_2) != 0:
            centroid_calculator(centroid_2)

        if len(centroid_3) != 0:
            centroid_calculator(centroid_3)

        if len(centroid_4) != 0:
            centroid_calculator(centroid_4)
    return centroids
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

# Open a text file for writing
with open(txt_filename, "w") as file:
    centroid_list = centroid_calculator(centroid_list)
file.close()
for alien , noofled ,x , y in centroid_list:
    img = cv2.circle(img, (round(float(x)),round(float(y))), 5, (0,255,0), -1)
    cv2.putText(img, f'Cluster of {noofled} leds', (round(float(x)+40),round(float(y)-20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 1, cv2.LINE_AA)
    cv2.putText(img, f'Type: {alien} ', (round(float(x)+40),round(float(y))), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0, 255), 1, cv2.LINE_AA)
    cv2.putText(img, f'({round(float(x), 2):.2f}, {round(float(y), 2):.2f})', (round(float(x)+40),round(float(y)+20)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
# Display the image with the drawn point
cv2.imshow('Image with Point', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
