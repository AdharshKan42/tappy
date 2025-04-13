import cv2
import numpy as np

# Load the main image and the template
main_image = cv2.imread('main.png')  # Replace with your full keyboard image
template = cv2.imread('template.png')

# Convert both to grayscale
main_gray = cv2.cvtColor(main_image, cv2.COLOR_BGR2GRAY)
template_gray = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)

# Perform template matching
result = cv2.matchTemplate(main_gray, template_gray, cv2.TM_CCOEFF_NORMED)

# Set a threshold to determine matches
threshold = 0.8
locations = np.where(result >= threshold)
print("Locations of matches:", locations)

# Draw rectangles around matched regions
w, h = template.shape[1], template.shape[0]
for pt in zip(*locations[::-1]):  # Switch columns and rows
    cv2.rectangle(main_image, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 2)

# Show the result
cv2.imshow('Detected Template', main_image)
while True:
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cv2.destroyAllWindows()