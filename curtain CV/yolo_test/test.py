from ultralytics import YOLO

model = YOLO("yolov8s-oiv7.pt")   # downloads automatically the first time
image_name = input("Enter image name (e.g. test.jpg): ").strip().strip('"')

results = model(image_name, conf=0.15)
#results[0].show()   # opens a window with boxes draw