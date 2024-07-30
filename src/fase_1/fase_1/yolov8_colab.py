from ultralytics import YOLO


model = YOLO("/home/pedro/Fase1_CBR/colab_model.pt")

results = model(source=0, show=True, conf=0.6, save=True)
