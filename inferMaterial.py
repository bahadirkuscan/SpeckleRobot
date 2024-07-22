import speckleRobotServer
import torch
from torchvision import transforms
from PIL import Image, ImageTk
import io
import tkinter as tk

# Set up the device
# device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")

def getImage(connection):
    # Receive image bytes and convert to both Tensor and PIL Image
    img_bytes = speckleRobotServer.receiveShot(connection)
    img_pil = Image.open(io.BytesIO(img_bytes))
    img_tensor = transform(img_pil).unsqueeze(0)  # Apply transformations and add batch dimension
    img_tensor = img_tensor.to(device)
    return img_tensor, img_pil.resize((224, 224))  # Resize PIL image for display

# Define image transformations
transform = transforms.Compose([
    transforms.Resize(256),
    transforms.CenterCrop(224),
    transforms.ToTensor(),
    transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
])

# Load model
model = torch.load('TrainedModels/model_new.pth', map_location=torch.device('cpu'))
# model.to(device)
model.eval()

# Initialize GUI
root = tk.Tk()
root.title("Class Prediction")
root.configure(background='black')

# Image display
image_label = tk.Label(root, borderwidth=2, relief="solid")
image_label.pack(side="left", padx=10, pady=10)

# Text display
text_label = tk.Label(root, text="Waiting for predictions...", font=('Helvetica', 16), fg="white", bg="black")
text_label.pack(side="right", padx=10, pady=10)

# Start server
server, connection, address = speckleRobotServer.startServer()

def update_gui(img_pil, prediction):
    img_tk = ImageTk.PhotoImage(img_pil)  # Convert PIL image to PhotoImage
    image_label.config(image=img_tk)
    image_label.image = img_tk  # Keep a reference

    class_names = {0: "Leather", 1: "Metal", 2: "Wood"}
    text_label.config(text=f"Predicted class: {prediction}")

while True:
    tensor_img, pil_img = getImage(connection)
    with torch.no_grad():
        outputs = model(tensor_img)
        _, predicted = torch.max(outputs, 1)
        update_gui(pil_img, predicted.item())
    root.update_idletasks()
    root.update()

root.mainloop()
