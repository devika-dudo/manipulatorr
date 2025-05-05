import cv2

img = cv2.imread("/home/devika/Pictures/Screenshot from 2025-03-11 15-54-51.png")  # Use any image
cv2.imshow("Test", img)
cv2.waitKey(0)
cv2.destroyAllWindows()

