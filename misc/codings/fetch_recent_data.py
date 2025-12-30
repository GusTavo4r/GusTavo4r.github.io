#!/usr/bin/env python3
"""
Incremental update script to fetch recent WakaTime data (last 3 days).
This script is designed to run daily via GitHub Actions to keep data up-to-date.
Features:
- Only fetches last 3 days of data
- Merges with existing data
- Lightweight and fast
"""

import urllib.request
import urllib.error
import base64
import json
import ssl
import os
import logging
from datetime import datetime, timedelta
from collections import OrderedDict

# Configuration
WAKATIME_API_KEY = 'waka_bbfc4972-29b1-47e2-bab0-a822624e7123'
API_BASE = 'https://api.wakatime.com/api/v1'
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
STATS_PATH = os.path.join(SCRIPT_DIR, 'wakatime_stats.json')
SUMMARIES_PATH = os.path.join(SCRIPT_DIR, 'wakatime_summaries.json')

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

def load_existing_data():
    """Load existing summaries data"""
    daily_data = OrderedDict()
    
    try:
        if os.path.exists(SUMMARIES_PATH):
            with open(SUMMARIES_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
            
            # Handle both old format (array) and new format (dict by date)
            if isinstance(data, dict):
                if 'daily_data' in data:
                    # New format: {"metadata": {...}, "daily_data": {"2022-01-01": {...}}}
                    daily_data = OrderedDict(sorted(data['daily_data'].items()))
                elif 'data' in data and isinstance(data['data'], list):
                    # Old format: {"data": [...]}
                    logger.info("Converting old format to new format...")
                    for day in data['data']:
                        date = day.get('range', {}).get('date')
                        if date:
                            daily_data[date] = extract_daily_info(day)
                elif all(isinstance(k, str) and len(k) == 10 and k.count('-') == 2 for k in data.keys() if k not in ['metadata', 'start', 'end']):
                    # Already in new format as dict (dates as keys at top level)
                    daily_data = OrderedDict(sorted([(k, v) for k, v in data.items() if k not in ['metadata', 'start', 'end']]))
            
            if daily_data:
                logger.info(f"📂 Loaded {len(daily_data)} days from existing data")
        
    except Exception as e:
        logger.warning(f"Could not load existing data: {e}")
    
    return daily_data

def extract_daily_info(day_data):
    """Extract essential daily information from WakaTime API response"""
    grand_total = day_data.get('grand_total', {})
    range_info = day_data.get('range', {})
    
    return {
        'date': range_info.get('date', ''),
        'total_seconds': grand_total.get('total_seconds', 0),
        'total_hours': round(grand_total.get('total_seconds', 0) / 3600, 2),
        'text': grand_total.get('text', '0 secs'),
        'languages': [
            {
                'name': lang.get('name', ''),
                'seconds': lang.get('total_seconds', 0),
                'percent': lang.get('percent', 0)
            }
            for lang in day_data.get('languages', [])[:10]  # Top 10 languages
        ],
        'editors': [
            {
                'name': editor.get('name', ''),
                'seconds': editor.get('total_seconds', 0),
                'percent': editor.get('percent', 0)
            }
            for editor in day_data.get('editors', [])[:5]  # Top 5 editors
        ],
        'projects': [
            {
                'name': proj.get('name', ''),
                'seconds': proj.get('total_seconds', 0),
                'percent': proj.get('percent', 0)
            }
            for proj in day_data.get('projects', [])[:5]  # Top 5 projects
        ]
    }

def save_daily_data(daily_data, metadata=None):
    """Save daily data in clean format"""
    output = {
        'metadata': metadata or {
            'last_updated': datetime.now().isoformat(),
            'total_days': len(daily_data),
            'date_range': {
                'start': min(daily_data.keys()) if daily_data else None,
                'end': max(daily_data.keys()) if daily_data else None
            }
        },
        'daily_data': daily_data
    }
    
    # Save with atomic write (write to temp file first)
    temp_path = SUMMARIES_PATH + '.tmp'
    with open(temp_path, 'w', encoding='utf-8') as f:
        json.dump(output, f, indent=2, ensure_ascii=False)
    
    # Atomic move
    os.replace(temp_path, SUMMARIES_PATH)

def fetch_recent_data(days=3):
    """Fetch recent WakaTime data (last N days)"""
    
    # Create SSL context
    ssl_context = ssl.create_default_context()
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE
    
    # Create auth header
    auth_string = f"{WAKATIME_API_KEY}:"
    auth_b64 = base64.b64encode(auth_string.encode()).decode()
    
    logger.info("=" * 60)
    logger.info("🔄 Starting Incremental WakaTime Data Update")
    logger.info("=" * 60)
    
    # 1. Get stats for last 30 days
    logger.info("📊 Fetching stats (last 30 days)...")
    stats_url = f"{API_BASE}/users/current/stats/last_30_days"
    
    try:
        req = urllib.request.Request(stats_url)
        req.add_header('Authorization', f'Basic {auth_b64}')
        
        with urllib.request.urlopen(req, timeout=30, context=ssl_context) as response:
            stats_data = json.loads(response.read().decode('utf-8'))
            with open(STATS_PATH, 'w', encoding='utf-8') as f:
                json.dump(stats_data, f, indent=2, ensure_ascii=False)
            logger.info(f"✅ Stats saved to {STATS_PATH}")
    except Exception as e:
        logger.error(f"❌ Error fetching stats: {e}")
        return False
    
    # 2. Load existing data
    daily_data = load_existing_data()
    
    # 3. Fetch recent days (last N days) using Summaries API
    # Note: Free accounts can access last 14 days, Premium can access all historical data
    end_date = datetime.now()
    start_date = end_date - timedelta(days=days - 1)  # Include today
    
    logger.info(f"📅 Fetching recent {days} days (from {start_date.strftime('%Y-%m-%d')} to {end_date.strftime('%Y-%m-%d')})...")
    
    start_str = start_date.strftime('%Y-%m-%d')
    end_str = end_date.strftime('%Y-%m-%d')
    url = f"{API_BASE}/users/current/summaries?start={start_str}&end={end_str}"
    
    try:
        req = urllib.request.Request(url)
        req.add_header('Authorization', f'Basic {auth_b64}')
        
        with urllib.request.urlopen(req, timeout=60, context=ssl_context) as response:
            data = json.loads(response.read().decode('utf-8'))
            
            if 'data' in data and data['data']:
                days_updated = 0
                days_added = 0
                
                for day in data['data']:
                    date = day.get('range', {}).get('date')
                    if date:
                        if date in daily_data:
                            days_updated += 1
                        else:
                            days_added += 1
                        daily_data[date] = extract_daily_info(day)
                
                # Sort by date
                daily_data = OrderedDict(sorted(daily_data.items()))
                
                # Save updated data
                save_daily_data(daily_data)
                
                logger.info("")
                logger.info("=" * 60)
                logger.info("✅ Incremental update completed!")
                logger.info("=" * 60)
                logger.info(f"📊 Update Statistics:")
                logger.info(f"   Days added: {days_added}")
                logger.info(f"   Days updated: {days_updated}")
                logger.info(f"   Total days in database: {len(daily_data)}")
                logger.info(f"   Date range: {min(daily_data.keys())} to {max(daily_data.keys())}")
                logger.info(f"   Saved to: {SUMMARIES_PATH}")
                logger.info("=" * 60)
                
                return True
            else:
                logger.warning("⚠️  No data returned from API")
                return False
                
    except urllib.error.HTTPError as e:
        # Handle Premium account requirement (402 Payment Required)
        if e.code == 402:
            # Free accounts can still access last 14 days, so if we're requesting within 14 days
            # and still get 402, it might be a different issue
            days_back = (end_date - start_date).days + 1
            if days_back <= 14:
                logger.warning("")
                logger.warning("=" * 60)
                logger.warning("⚠️  Summaries API returned 402 (Payment Required)")
                logger.warning("=" * 60)
                logger.warning(f"📝 Note: Requested {days_back} days (within free account's 14-day limit)")
                logger.warning("   This might indicate:")
                logger.warning("   1. API key issue, OR")
                logger.warning("   2. Account status changed, OR")
                logger.warning("   3. WakaTime API policy change")
                logger.warning("")
                logger.warning("   Stats data (last 30 days summary) has been updated successfully.")
                logger.warning("=" * 60)
            else:
                logger.warning("")
                logger.warning("=" * 60)
                logger.warning("⚠️  Summaries API requires Premium account (402 Payment Required)")
                logger.warning("=" * 60)
                logger.warning(f"📝 Note: Requested {days_back} days, but free accounts can only access last 14 days.")
                logger.warning("   Stats data (last 30 days summary) has been updated successfully.")
                logger.warning("   To update daily data beyond 14 days, you need Premium account.")
                logger.warning("=" * 60)
            # Stats data was already saved, so return True
            return True
        else:
            logger.error(f"❌ HTTP Error fetching recent data: {e.code} - {e.reason}")
            return False
    except urllib.error.URLError as e:
        logger.error(f"❌ URL Error fetching recent data: {e}")
        return False
    except Exception as e:
        logger.error(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == '__main__':
    import sys
    
    # Default to 3 days, but allow override via command line
    days_to_fetch = 3
    if len(sys.argv) > 1:
        try:
            days_to_fetch = int(sys.argv[1])
        except ValueError:
            logger.warning(f"Invalid days argument: {sys.argv[1]}, using default: 3")
    
    success = fetch_recent_data(days=days_to_fetch)
    sys.exit(0 if success else 1)

