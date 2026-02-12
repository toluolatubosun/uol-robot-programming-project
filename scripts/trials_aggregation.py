import json
import csv
import numpy as np
from pathlib import Path
from collections import defaultdict

# Configuration
WORLD_IDS = ['A', 'B']
NUM_TRIALS = 4
RESULTS_DIR = 'results'
MATCH_THRESHOLD = 3.0  # meters - max distance for valid match

def load_ground_truth(world_id):
    """Load ground truth objects for a world."""
    gt_path = Path(f"src/john_bot/data/worlds_description.json")
    
    with open(gt_path, 'r') as f:
        data = json.load(f)
    
    for world in data:
        if world['world_id'] == world_id:
            return world['objects']
    
    return []

def distance_2d(pos1, pos2):
    """Calculate 2D distance between positions."""
    dx = pos1['x'] - pos2['x']
    dy = pos1['y'] - pos2['y']
    return np.sqrt(dx**2 + dy**2)

def match_detections(detections, ground_truth):
    """
    Greedy matching: pair each detection with closest ground truth of same class.
    Only pairs within MATCH_THRESHOLD count as correct detections.
    """
    def get_class(class_id):
        return class_id.split(':')[1] if ':' in class_id else class_id
    
    # Build all possible pairs: (det_idx, gt_idx, distance)
    pairs = []
    for i, det in enumerate(detections):
        det_class = get_class(det['class_id'])
        for j, gt in enumerate(ground_truth):
            if gt['type'] == det_class:
                dist = distance_2d(det['position'], gt['position'])
                pairs.append((i, j, dist))
    
    # Sort by distance, match greedily
    pairs.sort(key=lambda x: x[2])
    
    matched_dets = set()
    matched_gts = set()
    correct_detections = []
    position_errors = []
    
    for det_idx, gt_idx, dist in pairs:
        if det_idx in matched_dets or gt_idx in matched_gts:
            continue
        
        # Only match if within threshold
        if dist <= MATCH_THRESHOLD:
            matched_dets.add(det_idx)
            matched_gts.add(gt_idx)
            correct_detections.append(detections[det_idx])
            position_errors.append(dist)
    
    # Unmatched detections = false detections
    false_detections = [d for i, d in enumerate(detections) if i not in matched_dets]
    
    # Unmatched ground truth = missed objects
    missed_objects_count = len(ground_truth) - len(matched_gts)
    
    return correct_detections, false_detections, missed_objects_count, position_errors

def calculate_trial_metrics(trial_json_path, ground_truth):
    """Calculate all metrics for a single trial."""
    with open(trial_json_path, 'r') as f:
        data = json.load(f)
    
    detections = data.get('detections', [])
    correct_dets, false_dets, missed_count, pos_errors = match_detections(detections, ground_truth)
    
    # Count per class
    def get_class(class_id):
        return class_id.split(':')[1] if ':' in class_id else class_id
    
    class_counts = defaultdict(int)
    for det in detections:
        class_counts[get_class(det['class_id'])] += 1
    
    # Calculate metrics
    correct_count = len(correct_dets)
    false_count = len(false_dets)
    total = len(detections)
    
    precision = correct_count / total if total > 0 else 0.0
    recall = correct_count / (correct_count + missed_count) if (correct_count + missed_count) > 0 else 0.0
    f1 = 2 * precision * recall / (precision + recall) if (precision + recall) > 0 else 0.0
    
    return {
        'trial_id': data.get('trial_id', 0),
        'total_detections': total,
        'fire_extinguisher_count': class_counts.get('FireExtinguisher', 0),
        'first_aid_box_count': class_counts.get('FirstAidBox', 0),
        'correct_detections': correct_count,
        'false_detections': false_count,
        'missed_objects_count': missed_count,
        'precision': round(precision, 3),
        'recall': round(recall, 3),
        'f1_score': round(f1, 3),
        'avg_confidence': round(np.mean([d['confidence'] for d in detections]), 3) if detections else 0.0,
        'avg_position_error_m': round(np.mean(pos_errors), 3) if pos_errors else 0.0,
        'exploration_time_s': round(data.get('exploration_duration_seconds', 0.0), 1),
        'verification_time_s': round(data.get('verification_duration_seconds', 0.0), 1),
        'total_time_s': round(data.get('exploration_duration_seconds', 0.0) + data.get('verification_duration_seconds', 0.0), 1),
        'map_coverage_pct': data.get('map_coverage_percentage', 0.0)
    }

def calculate_aggregates(metrics_list):
    """Calculate mean ± std across trials."""
    keys = [k for k in metrics_list[0].keys() if k != 'trial_id']
    agg = {'trial_id': 'Mean ± Std'}
    
    for key in keys:
        values = [m[key] for m in metrics_list]
        mean = np.mean(values)
        std = np.std(values)
        
        if key in ['precision', 'recall', 'f1_score', 'avg_confidence']:
            agg[key] = f"{mean:.3f} ± {std:.3f}"
        else:
            agg[key] = f"{mean:.1f} ± {std:.1f}"
    
    return agg

def process_world(world_id):
    """Process all trials for a world and generate CSV."""
    print(f"\nProcessing World {world_id}:")
    
    ground_truth = load_ground_truth(world_id)
    if not ground_truth:
        print(f"  No ground truth found, skipping")
        return
    
    metrics_list = []
    for trial_num in range(1, NUM_TRIALS + 1):
        trial_json = Path(f"{RESULTS_DIR}/exploration/World_{world_id}/trial_{trial_num}.json")
        
        if not trial_json.exists():
            print(f"  Trial {trial_num}: Not found, skipping")
            continue
        
        metrics = calculate_trial_metrics(trial_json, ground_truth)
        metrics_list.append(metrics)
        print(f"  Trial {trial_num}: ✓ (P={metrics['precision']:.2f}, R={metrics['recall']:.2f})")
    
    if not metrics_list:
        print(f"  No valid trials found")
        return
    
    # Write CSV
    csv_path = Path(f"{RESULTS_DIR}/summary/World_{world_id}_summary.csv")
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=metrics_list[0].keys())
        writer.writeheader()
        writer.writerows(metrics_list)
        writer.writerow(calculate_aggregates(metrics_list))
    
    print(f"Saved: {csv_path}")

def main():
    print("Starting statistical aggregation...")
    
    for world_id in WORLD_IDS:
        process_world(world_id)

if __name__ == '__main__':
    main()