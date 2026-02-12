import csv
import numpy as np
from pathlib import Path

# Configuration
WORLD_IDS = ['A', 'B']
RESULTS_DIR = 'results'

def load_world_trials(world_id):
    """Load trial data from a world summary CSV (excluding aggregate row)."""
    csv_path = Path(f"{RESULTS_DIR}/summary/World_{world_id}_summary.csv")
    
    if not csv_path.exists():
        return []
    
    trials = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            # Skip the aggregate row
            if row['trial_id'] == 'Mean ± Std':
                continue
            
            # Convert numeric fields
            numeric_row = {}
            for key, value in row.items():
                if key == 'trial_id':
                    numeric_row[key] = value
                else:
                    try:
                        numeric_row[key] = float(value)
                    except ValueError:
                        numeric_row[key] = value
            
            trials.append(numeric_row)
    
    return trials

def calculate_final_aggregate(all_trials):
    """Calculate mean ± std across all trials from all worlds."""
    if not all_trials:
        return {}
    
    keys = [k for k in all_trials[0].keys() if k != 'trial_id']
    agg = {'world_id': 'All', 'trial_count': len(all_trials)}
    
    for key in keys:
        values = [trial[key] for trial in all_trials]
        mean = np.mean(values)
        std = np.std(values)
        
        if key in ['precision', 'recall', 'f1_score', 'avg_confidence']:
            agg[key] = f"{mean:.3f} ± {std:.3f}"
        else:
            agg[key] = f"{mean:.1f} ± {std:.1f}"
    
    return agg

def main():
    print("Calculating final aggregate across all worlds...\n")
    
    # Collect all trials from all worlds
    all_trials = []
    for world_id in WORLD_IDS:
        trials = load_world_trials(world_id)
        if trials:
            print(f"World {world_id}: {len(trials)} trials loaded")
            all_trials.extend(trials)
        else:
            print(f"World {world_id}: No data found")
    
    if not all_trials:
        print("\nNo trial data found. Run aggregate_statistics.py first.")
        return
    
    print(f"\nTotal trials: {len(all_trials)}")
    
    # Calculate overall statistics
    final_agg = calculate_final_aggregate(all_trials)
    
    # Save to CSV
    csv_path = Path(f"{RESULTS_DIR}/summary/final_aggregate.csv")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=final_agg.keys())
        writer.writeheader()
        writer.writerow(final_agg)
    
    print(f"\nSaved: {csv_path}")
    print("Final aggregate complete!")

if __name__ == '__main__':
    main()