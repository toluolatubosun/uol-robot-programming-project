import os
import shutil
import yaml
from pathlib import Path

def combine_datasets():
    """Combines multiple YOLO datasets into one unified dataset."""
    
    base_dir = Path(__file__).parent
    output_dir = base_dir / "combined_dataset"
    
    # Define datasets to combine
    datasets = [
        {"path": base_dir.parent / "fire_extinguisher" / "fire_extinguisher_dataset", "keep_classes": ["FireExtinguisher"]},
        {"path": base_dir.parent / "first_aid_box" / "first_aid_box_dataset", "keep_classes": ["FirstAidBox"]}
    ]
    
    # Create output directories
    for split in ['train', 'valid', 'test']:
        (output_dir / split / 'images').mkdir(parents=True, exist_ok=True)
        (output_dir / split / 'labels').mkdir(parents=True, exist_ok=True)
    
    # Build class mapping
    new_classes = []
    class_mapping = {}
    
    for idx, dataset_info in enumerate(datasets):
        with open(dataset_info["path"] / "data.yaml", 'r') as f:
            old_classes = yaml.safe_load(f)['names']
        
        class_mapping[idx] = {}
        for old_id, name in enumerate(old_classes):
            if name in dataset_info['keep_classes']:
                if name not in new_classes:
                    new_classes.append(name)
                class_mapping[idx][old_id] = new_classes.index(name)
    
    print(f"Classes: {new_classes}\n")
    
    # Process datasets
    for idx, dataset_info in enumerate(datasets):
        dataset_path = dataset_info["path"]
        prefix = dataset_path.name.split('_')[0]
        
        for split in ['train', 'valid', 'test']:
            labels_dir = dataset_path / split / 'labels'
            images_dir = dataset_path / split / 'images'
            
            if not labels_dir.exists():
                continue
            
            for label_file in labels_dir.glob('*.txt'):
                # Filter and relabel
                new_lines = []
                for line in open(label_file):
                    parts = line.strip().split()
                    if parts and int(parts[0]) in class_mapping[idx]:
                        new_id = class_mapping[idx][int(parts[0])]
                        new_lines.append(f"{new_id} {' '.join(parts[1:])}\n")
                
                if new_lines:
                    # Copy files with prefix
                    img_name = label_file.stem + next(images_dir.glob(f"{label_file.stem}.*")).suffix
                    
                    shutil.copy(images_dir / img_name, 
                              output_dir / split / 'images' / f"{prefix}_{img_name}")
                    (output_dir / split / 'labels' / f"{prefix}_{label_file.name}").write_text(''.join(new_lines))
            
            print(f"{dataset_path.name}/{split}")
    
    # Create data.yaml
    yaml.dump({
        'train': '../train/images',
        'val': '../valid/images', 
        'test': '../test/images',
        'nc': len(new_classes),
        'names': new_classes
    }, open(output_dir / 'data.yaml', 'w'))
    
    print(f"\nCombined dataset: {output_dir}")

if __name__ == "__main__":
    combine_datasets()