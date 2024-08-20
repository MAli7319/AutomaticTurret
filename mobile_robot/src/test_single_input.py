from ultralytics import YOLO                  # type: ignore
import cv2                                    # type: ignore
import matplotlib.pyplot as plt               # type: ignore
import numpy as np                            # type: ignore


model = YOLO('best.pt')

img = cv2.imread("car2.jpg")

results = model(img)
cars = []
for result in results:
    boxes = result.boxes.cpu().numpy()
    for box in boxes:  # Iterate over boxes
        if box.cls[0] == 1 and box.conf[0] > 0.75:
            r = box.xyxy[0].astype(int)  # Get corner points as int
            cars.append(r)
            class_id = int(box.cls[0])  # Get class ID
            class_name = model.names[class_id]  # Get class name using the class ID
            cv2.rectangle(img, r[:2], r[2:], (0, 0, 255), 3)  # Draw boxes on the image

output = cv2.cvtColor((img), cv2.COLOR_BGR2RGB)
    

plt.imshow(output)
plt.axis('off')
plt.savefig("output.jpg", bbox_inches='tight', transparent=True, pad_inches=0, dpi=400)
plt.show()
