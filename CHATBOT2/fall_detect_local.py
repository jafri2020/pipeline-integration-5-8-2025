# watcher.py

import pandas as pd
import time
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
import os

class CSVWatcher(FileSystemEventHandler):
    def __init__(self, filepath, callback):
        self.filepath = filepath
        self.callback = callback
        self.last_df = pd.read_csv(filepath)

    def on_modified(self, event):
        if os.path.abspath(event.src_path) == os.path.abspath(self.filepath):
            try:
                current_df = pd.read_csv(self.filepath)
                if len(current_df) > len(self.last_df):
                    new_rows = current_df.iloc[len(self.last_df):]
                    self.last_df = current_df.copy()
                    self.callback(True, new_rows)  # Call the external function
                else:
                    self.callback(False, pd.DataFrame())
            except Exception as e:
                print("Error reading CSV:", e)

def start_watching(filepath, callback):
    directory = os.path.dirname(filepath)
    event_handler = CSVWatcher(filepath, callback)
    observer = Observer()
    observer.schedule(event_handler, path=directory or ".", recursive=False)
    observer.start()

    print(f"👀 Watching '{filepath}' for new rows...\nPress Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
