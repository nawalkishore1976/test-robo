import asyncio
from bleak import BleakScanner, BleakClient
import time
import csv
import json
from datetime import datetime
import os

class RoboLogCollector:
    def __init__(self, log_dir="logs"):
        self.log_dir = log_dir
        self.ensure_log_directory()
        
        # Create log files
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.txt_file = os.path.join(log_dir, f"robo_logs_{self.session_id}.txt")
        self.csv_file = os.path.join(log_dir, f"robo_logs_{self.session_id}.csv")
        self.json_file = os.path.join(log_dir, f"robo_logs_{self.session_id}.json")
        
        # Initialize files
        self.init_log_files()
        self.log_count = 0
        self.start_time = time.time()
        
    def ensure_log_directory(self):
        """Create logs directory if it doesn't exist"""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
    def init_log_files(self):
        """Initialize log files with headers"""
        # CSV header
        with open(self.csv_file, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(['PC_Timestamp', 'Arduino_Timestamp', 'Log_Counter', 'Level', 'Message', 'Session_ID'])
        
        # JSON array start
        with open(self.json_file, 'w', encoding='utf-8') as f:
            f.write('[\n')
            
        print(f"📁 Logs will be saved to:")
        print(f"  📄 TXT: {self.txt_file}")
        print(f"  📊 CSV: {self.csv_file}")  
        print(f"  🗂️  JSON: {self.json_file}\n")

    def parse_log_entry(self, raw_data):
        """Parse Arduino log entry: TIMESTAMP|COUNTER|LEVEL|MESSAGE"""
        try:
            data_str = raw_data.decode('utf-8').strip()
            if not data_str:
                return None
                
            parts = data_str.split('|', 3)  # Split into max 4 parts
            if len(parts) >= 4:
                return {
                    'arduino_timestamp': int(parts[0]),
                    'log_counter': int(parts[1]),
                    'level': parts[2],
                    'message': parts[3],
                    'pc_timestamp': time.time(),
                    'pc_datetime': datetime.now().isoformat(),
                    'session_id': self.session_id
                }
            else:
                # Handle malformed data
                return {
                    'arduino_timestamp': 0,
                    'log_counter': 0,
                    'level': 'RAW',
                    'message': data_str,
                    'pc_timestamp': time.time(),
                    'pc_datetime': datetime.now().isoformat(),
                    'session_id': self.session_id
                }
        except Exception as e:
            print(f"❌ Parse error: {e}")
            return None

    def save_log_entry(self, log_entry):
        """Save log entry to all formats"""
        if not log_entry:
            return
            
        self.log_count += 1
        
        # Console output with color coding
        level_colors = {
            'SYSTEM': '🔧',
            'CONNECTION': '🔗',
            'SENSOR': '📊',
            'STATUS': '✅',
            'BATTERY': '🔋',
            'WARNING': '⚠️',
            'ERROR': '❌',
            'RAW': '📝'
        }
        
        icon = level_colors.get(log_entry['level'], '📄')
        pc_time = datetime.fromtimestamp(log_entry['pc_timestamp']).strftime("%H:%M:%S")
        
        print(f"{icon} [{pc_time}] {log_entry['level']}: {log_entry['message']}")
        
        # Save to TXT file
        with open(self.txt_file, 'a', encoding='utf-8') as f:
            f.write(f"[{log_entry['pc_datetime']}] Arduino:{log_entry['arduino_timestamp']}ms "
                   f"#{log_entry['log_counter']} {log_entry['level']}: {log_entry['message']}\n")
        
        # Save to CSV file
        with open(self.csv_file, 'a', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow([
                log_entry['pc_datetime'],
                log_entry['arduino_timestamp'], 
                log_entry['log_counter'],
                log_entry['level'],
                log_entry['message'],
                log_entry['session_id']
            ])
        
        # Save to JSON file
        with open(self.json_file, 'a', encoding='utf-8') as f:
            if self.log_count > 1:
                f.write(',\n')
            json.dump(log_entry, f, indent=2)

    def finalize_logs(self):
        """Close JSON array and print summary"""
        with open(self.json_file, 'a', encoding='utf-8') as f:
            f.write('\n]')
            
        duration = time.time() - self.start_time
        print(f"\n📋 Session Summary:")
        print(f"   📊 Total logs collected: {self.log_count}")
        print(f"   ⏱️  Session duration: {duration:.1f} seconds")
        print(f"   💾 Files saved in: {self.log_dir}/")

    async def find_robo_device(self):
        """Find device with 'robo' in name"""
        print("🔍 Scanning for robo device...")
        
        while True:
            try:
                devices = await BleakScanner.discover(timeout=5.0)
                
                for device in devices:
                    if device.name and 'robo' in device.name.lower():
                        print(f"✅ Found: {device.name} [{device.address}]")
                        return device
                
                print("🔄 No robo device found, retrying...")
                await asyncio.sleep(2)
                
            except KeyboardInterrupt:
                return None

    def notification_handler(self, sender, data):
        """Handle incoming BLE notifications"""
        log_entry = self.parse_log_entry(data)
        self.save_log_entry(log_entry)

    async def collect_logs(self):
        """Main log collection loop"""
        print("🤖 Robo Log Collector")
        print("=" * 50)
        
        # Find device
        device = await self.find_robo_device()
        if not device:
            print("❌ No device found or scan cancelled")
            return
        
        # Connect and collect logs
        try:
            async with BleakClient(device.address, timeout=15.0) as client:
                print(f"🔗 Connected to {device.name}")
                
                # Find notification characteristic
                services = await client.get_services()
                notify_char = None
                
                for service in services:
                    for char in service.characteristics:
                        if 'notify' in char.properties:
                            notify_char = char.uuid
                            break
                    if notify_char:
                        break
                
                if notify_char:
                    await client.start_notify(notify_char, self.notification_handler)
                    print(f"🎧 Listening for logs... (Press Ctrl+C to stop)\n")
                    
                    # Keep collecting until interrupted
                    try:
                        while client.is_connected:
                            await asyncio.sleep(1)
                    except KeyboardInterrupt:
                        print("\n🛑 Stopping log collection...")
                        await client.stop_notify(notify_char)
                else:
                    print("❌ No notification characteristic found")
                    
        except Exception as e:
            print(f"❌ Connection error: {e}")
        
        finally:
            self.finalize_logs()

# Usage
async def main():
    collector = RoboLogCollector()
    await collector.collect_logs()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n👋 Goodbye!")