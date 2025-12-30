#!/usr/bin/env python3
"""
Verify WakaTime data format for statistics page
"""

import json
import os

def verify_data():
    # Files are in the same directory as this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    stats_path = os.path.join(script_dir, 'wakatime_stats.json')
    summaries_path = os.path.join(script_dir, 'wakatime_summaries.json')
    
    print("Verifying WakaTime data files...")
    print()
    
    # Check stats file
    try:
        with open(stats_path, 'r') as f:
            stats = json.load(f)
        
        if 'data' in stats and stats['data']:
            print("✓ wakatime_stats.json: Valid")
            print(f"  - Range: {stats['data'].get('range', 'N/A')}")
            print(f"  - Total seconds: {stats['data'].get('total_seconds', 0) / 3600:.2f} hours")
            print(f"  - Daily average: {stats['data'].get('daily_average', 0) / 3600:.2f} hours")
            if stats['data'].get('best_day'):
                print(f"  - Best day: {stats['data']['best_day'].get('date')} ({stats['data']['best_day'].get('total_seconds', 0) / 3600:.2f} hours)")
        else:
            print("✗ wakatime_stats.json: Invalid format")
            return False
    except Exception as e:
        print(f"✗ wakatime_stats.json: Error - {e}")
        return False
    
    print()
    
    # Check summaries file
    try:
        with open(summaries_path, 'r') as f:
            summaries = json.load(f)
        
        # Handle new format: {metadata: {...}, daily_data: {"2022-01-01": {...}}}
        daily_data = None
        if 'daily_data' in summaries:
            daily_data = summaries['daily_data']
            print("✓ wakatime_summaries.json: Valid (New format - Premium data)")
        elif 'data' in summaries:
            # Old format: array
            data_array = summaries['data']
            if isinstance(data_array, list) and len(data_array) > 0:
                print("✓ wakatime_summaries.json: Valid (Old format - Premium data)")
                # Convert to dict for processing
                daily_data = {}
                for day in data_array:
                    date = day.get('range', {}).get('date')
                    if date:
                        daily_data[date] = {
                            'total_seconds': day.get('grand_total', {}).get('total_seconds', 0),
                            'total_hours': day.get('grand_total', {}).get('total_seconds', 0) / 3600
                        }
            else:
                print("⚠ wakatime_summaries.json: Empty data array (Free account)")
                return True
        else:
            print("✗ wakatime_summaries.json: Invalid format")
            return False
        
        if daily_data:
            total_days = len(daily_data)
            print(f"  - Total days: {total_days}")
            
            # Count days with activity
            days_with_activity = [d for d in daily_data.values() if d.get('total_seconds', 0) > 0 or d.get('total_hours', 0) > 0]
            print(f"  - Days with coding activity: {len(days_with_activity)}")
            
            # Calculate total hours
            total_hours = sum([d.get('total_hours', 0) or (d.get('total_seconds', 0) / 3600) for d in daily_data.values()])
            print(f"  - Total hours: {total_hours:.2f}")
            
            # Find max day
            if days_with_activity:
                max_day = max(days_with_activity, key=lambda x: x.get('total_hours', 0) or (x.get('total_seconds', 0) / 3600))
                max_hours = max_day.get('total_hours', 0) or (max_day.get('total_seconds', 0) / 3600)
                # Find the date for max day
                max_date = None
                for date, day in daily_data.items():
                    if day == max_day or (day.get('total_hours', 0) or (day.get('total_seconds', 0) / 3600)) == max_hours:
                        max_date = date
                        break
                print(f"  - Max hours in a day: {max_hours:.2f} ({max_date or 'N/A'})")
            
            # Check date range
            if daily_data:
                dates = sorted(daily_data.keys())
                print(f"  - Date range: {dates[0]} to {dates[-1]}")
            
            # Show metadata if available
            if 'metadata' in summaries:
                meta = summaries['metadata']
                if 'last_updated' in meta:
                    print(f"  - Last updated: {meta['last_updated']}")
    except Exception as e:
        print(f"✗ wakatime_summaries.json: Error - {e}")
        return False
    
    print()
    print("✓ All data files are valid!")
    print()
    print("Next steps:")
    print("1. Start local server: ./misc/codings/start_test_server.sh")
    print("2. Open http://localhost:8000/statistics.html in your browser")
    print("3. You should see a cumulative coding time curve chart")
    
    return True

if __name__ == '__main__':
    verify_data()

