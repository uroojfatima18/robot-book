#!/usr/bin/env python3
"""
Log Viewer Utility

Chapter 5: Adaptive Robotics
Tool for viewing and analyzing decision log files.

Features:
- View log entries with filtering
- Search by behavior, trigger, or time range
- Generate statistics and reports
- Detect anomalies

Usage:
    # View all entries
    python log_viewer.py logs/decision_log_2025-12-30.json

    # Filter by behavior
    python log_viewer.py logs/decision_log.json --behavior avoid

    # Show summary statistics
    python log_viewer.py logs/decision_log.json --summary

    # Find anomalies
    python log_viewer.py logs/decision_log.json --anomalies
"""

import json
import argparse
from datetime import datetime, timedelta
from typing import List, Dict, Optional
from collections import Counter


def load_log_file(filepath: str) -> List[Dict]:
    """
    Load entries from a JSON lines log file.

    Args:
        filepath: Path to the log file

    Returns:
        List of log entry dictionaries
    """
    entries = []
    with open(filepath, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if line:
                try:
                    entries.append(json.loads(line))
                except json.JSONDecodeError as e:
                    print(f"Warning: Invalid JSON on line {line_num}: {e}")
    return entries


def filter_by_behavior(entries: List[Dict], behavior: str) -> List[Dict]:
    """Filter entries by target behavior."""
    return [
        e for e in entries
        if e['decision']['to_behavior'] == behavior
    ]


def filter_by_trigger(entries: List[Dict], trigger: str) -> List[Dict]:
    """Filter entries by trigger rule."""
    return [
        e for e in entries
        if e['decision']['trigger_rule'] == trigger
    ]


def filter_by_time_range(entries: List[Dict],
                         start: Optional[str] = None,
                         end: Optional[str] = None) -> List[Dict]:
    """
    Filter entries by time range.

    Args:
        entries: List of log entries
        start: ISO format start time (inclusive)
        end: ISO format end time (inclusive)

    Returns:
        Filtered list of entries
    """
    filtered = entries

    if start:
        start_dt = datetime.fromisoformat(start)
        filtered = [
            e for e in filtered
            if datetime.fromisoformat(e['timestamp']) >= start_dt
        ]

    if end:
        end_dt = datetime.fromisoformat(end)
        filtered = [
            e for e in filtered
            if datetime.fromisoformat(e['timestamp']) <= end_dt
        ]

    return filtered


def filter_switches_only(entries: List[Dict]) -> List[Dict]:
    """Get only entries where behavior actually changed."""
    return [
        e for e in entries
        if e['decision']['from_behavior'] != e['decision']['to_behavior']
    ]


def filter_invalid_sensors(entries: List[Dict]) -> List[Dict]:
    """Get entries with invalid sensor data."""
    return [
        e for e in entries
        if not e['input'].get('valid', True)
    ]


def print_entry(entry: Dict, verbose: bool = False):
    """
    Print a single log entry.

    Args:
        entry: Log entry dictionary
        verbose: If True, show additional details
    """
    ts = entry['timestamp'].split('T')[1][:12]  # HH:MM:SS.mmm
    seq = entry['sequence']
    from_b = entry['decision']['from_behavior']
    to_b = entry['decision']['to_behavior']
    value = entry['input']['value']
    valid = entry['input'].get('valid', True)

    # Indicate if sensor invalid
    valid_indicator = "" if valid else " [INVALID]"

    # Indicate if behavior changed
    change_indicator = " *" if from_b != to_b else ""

    print(f"[{ts}] #{seq:4d}: {from_b:10s} -> {to_b:10s} "
          f"(value: {value:6.3f}){valid_indicator}{change_indicator}")

    if verbose:
        print(f"         Trigger: {entry['decision']['trigger_rule']}")
        print(f"         Thresholds: activate={entry['decision']['activate_threshold']}, "
              f"deactivate={entry['decision']['deactivate_threshold']}")
        if entry.get('outcome'):
            outcome = entry['outcome']
            print(f"         Command: linear={outcome['command_linear_x']:.2f}, "
                  f"angular={outcome['command_angular_z']:.2f}")


def print_summary(entries: List[Dict]):
    """Print summary statistics for log entries."""
    if not entries:
        print("No entries to summarize.")
        return

    # Basic counts
    total = len(entries)
    switches = len(filter_switches_only(entries))
    invalid = len(filter_invalid_sensors(entries))

    # Behavior distribution
    behaviors = Counter(e['decision']['to_behavior'] for e in entries)

    # Trigger distribution
    triggers = Counter(e['decision']['trigger_rule'] for e in entries)

    # Time range
    timestamps = [datetime.fromisoformat(e['timestamp']) for e in entries]
    duration = max(timestamps) - min(timestamps)

    print("\n" + "=" * 60)
    print("LOG SUMMARY")
    print("=" * 60)
    print(f"Total entries:       {total}")
    print(f"Behavior switches:   {switches} ({100*switches/total:.1f}%)")
    print(f"Invalid sensors:     {invalid} ({100*invalid/total:.1f}%)")
    print(f"Time span:           {duration}")
    print(f"Avg decisions/sec:   {total / max(duration.total_seconds(), 1):.2f}")

    print("\nBehavior Distribution:")
    print("-" * 40)
    for behavior, count in behaviors.most_common():
        pct = 100 * count / total
        bar = "#" * int(pct / 2)
        print(f"  {behavior:15s}: {count:5d} ({pct:5.1f}%) {bar}")

    print("\nTrigger Distribution:")
    print("-" * 40)
    for trigger, count in triggers.most_common():
        print(f"  {trigger:20s}: {count:5d}")

    print("=" * 60)


def find_anomalies(entries: List[Dict]) -> List[Dict]:
    """
    Find anomalous entries in the log.

    Anomalies include:
    - Invalid sensor readings
    - Rapid switching (< 100ms between switches)
    - Same behavior for extended period with no sensor change

    Args:
        entries: List of log entries

    Returns:
        List of anomaly reports
    """
    anomalies = []

    prev_entry = None
    prev_switch_time = None

    for entry in entries:
        ts = datetime.fromisoformat(entry['timestamp'])

        # Check for invalid sensor
        if not entry['input'].get('valid', True):
            anomalies.append({
                'type': 'INVALID_SENSOR',
                'sequence': entry['sequence'],
                'timestamp': entry['timestamp'],
                'description': f"Sensor reported invalid data (value={entry['input']['value']})"
            })

        # Check for rapid switching
        if entry['decision']['from_behavior'] != entry['decision']['to_behavior']:
            if prev_switch_time:
                delta = ts - prev_switch_time
                if delta.total_seconds() < 0.1:  # Less than 100ms
                    anomalies.append({
                        'type': 'RAPID_SWITCH',
                        'sequence': entry['sequence'],
                        'timestamp': entry['timestamp'],
                        'description': f"Switch occurred {delta.total_seconds()*1000:.0f}ms after previous switch"
                    })
            prev_switch_time = ts

        prev_entry = entry

    return anomalies


def print_anomalies(anomalies: List[Dict]):
    """Print detected anomalies."""
    if not anomalies:
        print("\nNo anomalies detected.")
        return

    print("\n" + "=" * 60)
    print(f"ANOMALIES DETECTED: {len(anomalies)}")
    print("=" * 60)

    # Group by type
    by_type = {}
    for a in anomalies:
        atype = a['type']
        if atype not in by_type:
            by_type[atype] = []
        by_type[atype].append(a)

    for atype, items in by_type.items():
        print(f"\n{atype} ({len(items)} occurrences):")
        print("-" * 40)
        for item in items[:5]:  # Show first 5
            print(f"  #{item['sequence']} @ {item['timestamp'].split('T')[1][:8]}")
            print(f"    {item['description']}")
        if len(items) > 5:
            print(f"  ... and {len(items) - 5} more")

    print("=" * 60)


def main():
    """Main entry point for log viewer."""
    parser = argparse.ArgumentParser(
        description='View and analyze decision log files',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s logs/decision_log.json
  %(prog)s logs/decision_log.json --behavior avoid
  %(prog)s logs/decision_log.json --summary
  %(prog)s logs/decision_log.json --anomalies
  %(prog)s logs/decision_log.json --switches-only --verbose
        """
    )

    parser.add_argument('logfile', help='Path to log file')
    parser.add_argument('--behavior', '-b', help='Filter by behavior name')
    parser.add_argument('--trigger', '-t', help='Filter by trigger rule')
    parser.add_argument('--start', help='Filter by start time (ISO format)')
    parser.add_argument('--end', help='Filter by end time (ISO format)')
    parser.add_argument('--switches-only', '-w', action='store_true',
                        help='Show only entries where behavior changed')
    parser.add_argument('--invalid-only', '-i', action='store_true',
                        help='Show only entries with invalid sensor data')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show detailed entry information')
    parser.add_argument('--summary', '-s', action='store_true',
                        help='Show summary statistics')
    parser.add_argument('--anomalies', '-a', action='store_true',
                        help='Detect and show anomalies')
    parser.add_argument('--limit', '-n', type=int, default=0,
                        help='Limit number of entries shown')

    args = parser.parse_args()

    # Load log file
    try:
        entries = load_log_file(args.logfile)
    except FileNotFoundError:
        print(f"Error: File not found: {args.logfile}")
        return 1
    except Exception as e:
        print(f"Error loading file: {e}")
        return 1

    print(f"Loaded {len(entries)} entries from {args.logfile}")

    # Apply filters
    if args.behavior:
        entries = filter_by_behavior(entries, args.behavior)
        print(f"Filtered to {len(entries)} entries with behavior '{args.behavior}'")

    if args.trigger:
        entries = filter_by_trigger(entries, args.trigger)
        print(f"Filtered to {len(entries)} entries with trigger '{args.trigger}'")

    if args.start or args.end:
        entries = filter_by_time_range(entries, args.start, args.end)
        print(f"Filtered to {len(entries)} entries in time range")

    if args.switches_only:
        entries = filter_switches_only(entries)
        print(f"Filtered to {len(entries)} behavior switches")

    if args.invalid_only:
        entries = filter_invalid_sensors(entries)
        print(f"Filtered to {len(entries)} invalid sensor entries")

    # Output
    if args.summary:
        print_summary(entries)

    if args.anomalies:
        anomalies = find_anomalies(entries)
        print_anomalies(anomalies)

    # Print entries (unless only summary/anomalies requested)
    if not (args.summary or args.anomalies) or args.verbose:
        print("\n" + "-" * 60)
        limit = args.limit if args.limit > 0 else len(entries)
        for entry in entries[:limit]:
            print_entry(entry, args.verbose)

        if args.limit > 0 and len(entries) > args.limit:
            print(f"\n... showing {args.limit} of {len(entries)} entries")

    return 0


if __name__ == '__main__':
    exit(main())
