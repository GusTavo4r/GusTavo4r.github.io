#!/usr/bin/env python3
"""
Script to fetch WakaTime data and save as JSON files.
Features:
- Progress bar with tqdm
- Resume from existing data
- Clean daily-based JSON structure
"""

import urllib.request
import urllib.error
import base64
import json
import ssl
import time
import os
import logging
from datetime import datetime, timedelta
from collections import OrderedDict

try:
    from tqdm import tqdm
except ImportError:
    print("⚠️  tqdm not installed. Installing...")
    import subprocess
    subprocess.check_call(['pip3', 'install', 'tqdm', '-q'])
    from tqdm import tqdm

# Configuration
# API Key should be set via environment variable WAKATIME_API_KEY
# For local use, you can set it in your shell: export WAKATIME_API_KEY='your-key-here'
WAKATIME_API_KEY = os.environ.get('WAKATIME_API_KEY', '')
if not WAKATIME_API_KEY:
    raise ValueError("WAKATIME_API_KEY environment variable is required. Please set it before running this script.")
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
    """Load existing summaries data for resume"""
    daily_data = OrderedDict()
    last_date = None
    
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
                else:
                    # Try to extract dates from any structure
                    logger.warning("Unknown data format, attempting to parse...")
                    daily_data = OrderedDict()
            
            if daily_data:
                last_date = max(daily_data.keys())
                logger.info(f"📂 Loaded {len(daily_data)} days from existing data")
                logger.info(f"📅 Last date: {last_date}")
        
    except Exception as e:
        logger.warning(f"Could not load existing data: {e}")
    
    return daily_data, last_date

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

def fetch_chunk(auth_b64, ssl_context, start_date, end_date, max_retries=3):
    """Fetch a chunk of data with retry logic"""
    start_str = start_date.strftime('%Y-%m-%d')
    end_str = end_date.strftime('%Y-%m-%d')
    url = f"{API_BASE}/users/current/summaries?start={start_str}&end={end_str}"
    
    for attempt in range(max_retries):
        try:
            req = urllib.request.Request(url)
            req.add_header('Authorization', f'Basic {auth_b64}')
            
            with urllib.request.urlopen(req, timeout=120, context=ssl_context) as response:
                data = json.loads(response.read().decode('utf-8'))
                if 'data' in data and data['data']:
                    return data['data']
                return []
        except (urllib.error.HTTPError, urllib.error.URLError) as e:
            if attempt < max_retries - 1:
                wait_time = (attempt + 1) * 2
                logger.warning(f"  ⚠️  Error fetching {start_str} to {end_str}: {str(e)[:50]}... Retry {attempt + 1}/{max_retries} in {wait_time}s")
                time.sleep(wait_time)
            else:
                logger.error(f"  ✗ Failed to fetch {start_str} to {end_str} after {max_retries} attempts")
                return None
        except Exception as e:
            logger.error(f"  ✗ Unexpected error: {e}")
            return None
    
    return None

def fetch_wakatime_data():
    """Fetch WakaTime stats and summaries data"""
    
    # Create SSL context
    ssl_context = ssl.create_default_context()
    ssl_context.check_hostname = False
    ssl_context.verify_mode = ssl.CERT_NONE
    
    # Create auth header
    auth_string = f"{WAKATIME_API_KEY}:"
    auth_b64 = base64.b64encode(auth_string.encode()).decode()
    
    logger.info("=" * 60)
    logger.info("🚀 Starting WakaTime Data Fetch")
    logger.info("=" * 60)
    
    # 1. Get stats for last 30 days
    logger.info("📊 Fetching stats (last 30 days)...")
    stats_url = f"{API_BASE}/users/current/stats/last_30_days"
    stats_data = None
    
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
    
    # 2. Get all daily summaries from 2022 to now
    logger.info("📅 Fetching daily summaries from 2022...")
    end_date = datetime.now()
    start_date = datetime(2022, 1, 1)
    
    # Load existing data for resume
    daily_data, last_date = load_existing_data()
    
    if last_date:
        try:
            resume_date = datetime.strptime(last_date, '%Y-%m-%d') + timedelta(days=1)
            if resume_date < end_date:
                logger.info(f"🔄 Resuming from {last_date}, continuing from {resume_date.strftime('%Y-%m-%d')}")
                current_start = resume_date
            else:
                logger.info("✅ All data already fetched!")
                return True
        except:
            current_start = start_date
    else:
        current_start = start_date
    
    # Calculate total days to fetch
    total_days_needed = (end_date - current_start).days
    if total_days_needed <= 0:
        logger.info("✅ All data already fetched!")
        return True
    
    logger.info(f"📈 Total days to fetch: {total_days_needed} (from {current_start.strftime('%Y-%m-%d')} to {end_date.strftime('%Y-%m-%d')})")
    
    # Fetch in chunks
    chunk_days = 30
    total_chunks = (total_days_needed + chunk_days - 1) // chunk_days
    
    logger.info(f"📦 Fetching in {total_chunks} chunks ({chunk_days} days per chunk)")
    logger.info("")
    
    # Use tqdm for progress bar
    pbar = tqdm(total=total_days_needed, desc="Fetching days", unit="day", ncols=100)
    
    chunk_num = 0
    save_interval = 5  # Save every 5 chunks
    
    try:
        while current_start < end_date:
            chunk_num += 1
            current_end = min(current_start + timedelta(days=chunk_days - 1), end_date)
            
            chunk_start_str = current_start.strftime('%Y-%m-%d')
            chunk_end_str = current_end.strftime('%Y-%m-%d')
            
            logger.info(f"[{chunk_num}/{total_chunks}] Fetching {chunk_start_str} to {chunk_end_str}...")
            
            # Fetch chunk
            chunk_data = fetch_chunk(auth_b64, ssl_context, current_start, current_end)
            
            if chunk_data is None:
                logger.warning(f"  ⚠️  Skipping chunk {chunk_num}, continuing...")
                current_start = current_end + timedelta(days=1)
                continue
            
            # Process and save daily data
            days_added = 0
            for day in chunk_data:
                date = day.get('range', {}).get('date')
                if date:
                    daily_data[date] = extract_daily_info(day)
                    days_added += 1
                    pbar.update(1)
            
            logger.info(f"  ✅ Got {days_added} days (Total: {len(daily_data)} days)")
            
            # Save progress periodically
            if chunk_num % save_interval == 0:
                save_daily_data(daily_data)
                logger.info(f"  💾 Progress saved: {len(daily_data)} days")
            
            current_start = current_end + timedelta(days=1)
            time.sleep(1)  # Rate limiting
        
        # Final save
        pbar.close()
        save_daily_data(daily_data)
        
        # Calculate statistics
        total_hours = sum([day.get('total_hours', 0) for day in daily_data.values()])
        days_with_activity = len([d for d in daily_data.values() if d.get('total_seconds', 0) > 0])
        
        logger.info("")
        logger.info("=" * 60)
        logger.info("✅ Data fetch completed!")
        logger.info("=" * 60)
        logger.info(f"📊 Statistics:")
        logger.info(f"   Total days: {len(daily_data)}")
        logger.info(f"   Days with activity: {days_with_activity}")
        logger.info(f"   Total coding hours: {total_hours:.2f}")
        logger.info(f"   Date range: {min(daily_data.keys())} to {max(daily_data.keys())}")
        logger.info(f"   Saved to: {SUMMARIES_PATH}")
        logger.info("=" * 60)
        
        return True
        
    except KeyboardInterrupt:
        pbar.close()
        logger.warning("\n⚠️  Interrupted by user. Saving progress...")
        save_daily_data(daily_data)
        logger.info(f"💾 Progress saved: {len(daily_data)} days")
        return False
    except Exception as e:
        pbar.close()
        logger.error(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        save_daily_data(daily_data)
        logger.info(f"💾 Progress saved: {len(daily_data)} days")
        return False

if __name__ == '__main__':
    success = fetch_wakatime_data()
    exit(0 if success else 1)
