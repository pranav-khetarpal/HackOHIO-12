import cv2

max_tested = 10  # Increase the range if necessary

for i in range(max_tested):
    print(f"Testing camera index {i}")
    cap = cv2.VideoCapture(i, cv2.CAP_AVFOUNDATION)

    if not cap.isOpened():
        print(f"Camera index {i} is not available.")
        continue
    ret, frame = cap.read()
    if ret:
        print(f"Camera index {i} works!")
        cv2.imshow(f"Camera {i}", frame)
        cv2.waitKey(1000)  # Display for 1 second
        cap.release()
        cv2.destroyAllWindows()
    else:
        print(f"Camera index {i} does not return a frame.")
        cap.release()
