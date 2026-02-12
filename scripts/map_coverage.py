import json
import numpy as np
from PIL import Image
from pathlib import Path

# Configuration
WORLD_IDS = ['A', 'B']
NUM_TRIALS = 4
RESULTS_DIR = 'results'

def calculate_coverage(pgm_path):
    """Calculate coverage from PGM occupancy grid."""
    img = Image.open(pgm_path)
    grid = np.array(img)
    
    # ROS map_saver standard values:
    # 254 = free space (explored)
    # 205 = unknown (unexplored)  
    # 0 = occupied (walls)
    
    free_cells = np.sum(grid == 254)
    unknown_cells = np.sum(grid == 205)
    
    explorable_cells = free_cells + unknown_cells
    coverage = (free_cells / explorable_cells * 100) if explorable_cells > 0 else 0.0
    
    return round(coverage, 2)

def main():
    print("Starting map coverage calculation...\n")
    
    for world_id in WORLD_IDS:
        print(f"Processing World {world_id}:")
        
        for trial_num in range(1, NUM_TRIALS + 1):
            # Paths
            map_pgm = Path(f"{RESULTS_DIR}/maps/World_{world_id}/trial_{trial_num}.pgm")
            trial_json = Path(f"{RESULTS_DIR}/exploration/World_{world_id}/trial_{trial_num}.json")
            
            # Check if files exist
            if not map_pgm.exists():
                print(f"  Trial {trial_num}: Map not found, skipping")
                continue
            
            if not trial_json.exists():
                print(f"  Trial {trial_num}: JSON not found, skipping")
                continue
            
            # Calculate coverage
            coverage = calculate_coverage(map_pgm)
            
            # Update JSON
            with open(trial_json, 'r') as f:
                data = json.load(f)
            
            data['map_coverage_percentage'] = coverage
            
            with open(trial_json, 'w') as f:
                json.dump(data, f, indent=4)
            
            print(f"  Trial {trial_num}: {coverage}% coverage ✓")
        
        print()

if __name__ == '__main__':
    main()