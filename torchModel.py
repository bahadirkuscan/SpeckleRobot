import os
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from torchvision import models, transforms, datasets
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import LabelEncoder
from PIL import Image
from tqdm import tqdm
from sklearn.metrics import confusion_matrix
import matplotlib.pyplot as plt
import seaborn as sns


# Define constants
dataset_path = 'D:\General Projects\Python Projects\Engineering_Project\SpeckleRobotDataset'
img_height, img_width = 224, 224
num_classes = 14
batch_size = 64
num_epochs = 5
save_path = 'TrainedModels\model_new.pth'

# Define data preprocessing and augmentation transforms
data_transforms = {
    'train': transforms.Compose([
        transforms.RandomResizedCrop(img_height),
        transforms.RandomHorizontalFlip(),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ]),
    'val': transforms.Compose([
        transforms.Resize((img_height, img_width)),
        transforms.CenterCrop(img_height),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ]),
}

# Custom dataset class
class CustomDataset(Dataset):
    def __init__(self, file_paths, labels, transform=None):
        self.file_paths = file_paths
        self.labels = labels
        self.transform = transform

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx):
        img = Image.open(self.file_paths[idx]).convert('RGB')
        label = self.labels[idx]
        if self.transform:
            img = self.transform(img)
        return img, label

# Load images and labels
file_paths = []
labels = []
for folder_name in tqdm(os.listdir(dataset_path)):
    folder_path = os.path.join(dataset_path, folder_name)
    if os.path.isdir(folder_path):
        for file_name in os.listdir(folder_path):
            file_path = os.path.join(folder_path, file_name)
            if not file_path.endswith('.txt'):
                file_paths.append(file_path)
                labels.append(folder_name)

# Encode labels
print("Finished loading images and labels.")
encoder = LabelEncoder()
labels_encoded = encoder.fit_transform(labels)

# Split dataset
file_paths_train, file_paths_val, labels_train, labels_val = train_test_split(
    file_paths, labels_encoded, test_size=0.2, random_state=42 ,stratify=labels_encoded)

# Create custom datasets and data loaders
train_dataset = CustomDataset(file_paths_train, labels_train, transform=data_transforms['train'])
val_dataset = CustomDataset(file_paths_val, labels_val, transform=data_transforms['val'])

train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)

# Load pre-trained ResNet model
model = models.resnet50(pretrained=True)
num_ftrs = model.fc.in_features
model.fc = nn.Linear(num_ftrs, num_classes)

# Freeze convolutional layers
for param in model.parameters():
    param.requires_grad = False

# Unfreeze parameters of the fully connected layer
for param in model.fc.parameters():
    param.requires_grad = True

# Define loss function and optimizer
criterion = nn.CrossEntropyLoss()
optimizer = optim.Adam(model.parameters(), lr=0.003)  # Optimize all parameters

# Move model to GPU if available
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
model = model.to(device)

# Training loop
for epoch in tqdm(range(num_epochs)):
    model.train()
    running_loss = 0.0
    for inputs, labels in tqdm(train_loader):
        inputs, labels = inputs.to(device), labels.to(device)
        optimizer.zero_grad()
        outputs = model(inputs)
        loss = criterion(outputs, labels)
        loss.backward()
        optimizer.step()
        running_loss += loss.item() * inputs.size(0)
    epoch_loss = running_loss / len(train_dataset)

    # Validation loop
    model.eval()
    correct = 0
    total = 0
    val_predictions = []
    val_targets = []
    with torch.no_grad():
        for inputs, labels in val_loader:
            inputs, labels = inputs.to(device), labels.to(device)
            outputs = model(inputs)
            _, predicted = torch.max(outputs, 1)
            total += labels.size(0)
            correct += (predicted == labels).sum().item()
            val_predictions.extend(predicted.cpu().numpy())
            val_targets.extend(labels.cpu().numpy())
    val_accuracy = correct / total

    print(f'Epoch [{epoch+1}/{num_epochs}], '
          f'Training Loss: {epoch_loss:.4f}, '
          f'Validation Accuracy: {val_accuracy:.4f}')

# Compute confusion matrix
conf_matrix = confusion_matrix(val_targets, val_predictions)
print("Confusion Matrix:")
print(conf_matrix)

print('Training finished.')


torch.save(model, save_path)
# Plot confusion matrix as heatmap
plt.figure(figsize=(10, 8))
sns.heatmap(conf_matrix, annot=True, fmt="d", cmap="Blues", cbar=False)
plt.xlabel("Predicted Label")
plt.ylabel("True Label")
plt.title("Confusion Matrix")
plt.show()

print('Training finished.')