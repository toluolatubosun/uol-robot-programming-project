import os
from roboflow import Roboflow
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv(os.path.join(os.path.dirname(__file__), '..', '.env'))

api_key = os.getenv('ROBOFLOW_API_KEY')

# Change working directory to the script's directory
os.chdir(os.path.dirname(os.path.abspath(__file__)))

rf = Roboflow(api_key=api_key)
project = rf.workspace("vatly").project("fire-extinguisher-deefc")
dataset = project.version(2).download("yolov8")

print(f"Dataset downloaded to: {dataset.location}")
