import os
import torch
import subprocess

# Change working directory to the script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# Run the YOLO training command - will auto-detect and use GPU if available
device = '0' if torch.cuda.is_available() else 'cpu'
command = f"yolo train model=yolov8m.pt data=fire_extinguisher_dataset/data.yaml epochs=50 imgsz=640 device={device} workers=0"
print(f"Working directory: {os.getcwd()}")
print(f"Using device: {'GPU (CUDA)' if device == '0' else 'CPU'}")

if device == '0':
    print(f"GPU: {torch.cuda.get_device_name(0)}")
    cudnn_status = "enabled" if torch.backends.cudnn.enabled else "disabled"
    print(f"cuDNN: {cudnn_status}")

print(f"Running command: {command}")
subprocess.run(command, shell=True, check=True)
