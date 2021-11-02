from pytorchyolo import detect, models
import cv2

model = models.load_model(
  "yolov3-spp-6cls.cfg", 
  "best_model_12.pt")


  # Load the image as a numpy array
vidcap = cv2.VideoCapture('_out/my_video.mp4')
success,image = vidcap.read()
count = 0
while success:
    # Convert OpenCV bgr to rgb
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    boxes = detect.detect_image(model, img)
    success,image = vidcap.read()
#   print('Read a new frame: ', success)
    count += 1
    print(boxes)
    input()

print(count)



# Runs the YOLO model on the image 
boxes = detect.detect_image(model, img)

print(boxes)