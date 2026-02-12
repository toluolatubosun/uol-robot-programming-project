import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'john_bot'

# Function to gather all files recursively
def get_all_files_in_folder(folder):
    data_files = []
    
    if not os.path.exists(folder):
        return data_files
    
    # Walk through all directories in the models folder
    for root, _dirs, files in os.walk(folder):
        if files:
            # Calculate the relative path from the models directory
            rel_path = os.path.relpath(root, '.')
            install_path = os.path.join('share', package_name, rel_path)
            
            # Get all files in this directory
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_list))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'data'), glob('data/*.json')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        (os.path.join('share', package_name, 'yolo_model_weights'), glob('yolo_model_weights/*.pt')),
    ] + get_all_files_in_folder('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='John Olatubosun ',
    maintainer_email='toluolatubosun@gmail.com',
    description='LIMO robot emergency equipment inventory system',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = john_bot.object_detection:main',
            'results_manager = john_bot.results_manager:main',
            'exploration_monitor = john_bot.exploration_monitor:main',
            'verification_controller = john_bot.verification_controller:main',
        ],
    },
)
