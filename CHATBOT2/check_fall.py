import pandas as pd
import os
 
def append_row_to_csv(file_path, row_dict):
    # If file does not exist, create it with headers
    if not os.path.exists(file_path):
        df = pd.DataFrame([row_dict])
        df.to_csv(file_path, index=False)
    else:
        df = pd.DataFrame([row_dict])
        df.to_csv(file_path, mode='a', index=False, header=False)
 
# Example usage
new_row = {
    'timestamp': '2025-06-20 14:30:00',
    'event': 'Fall Detected',
    'location': 'Room'
}
 
append_row_to_csv(r"/home/nvidia/AI_Results/FALL/fall_results.csv", new_row)
# append_row_to_csv(r"E:\Conversational_Chatbot\Conversational Chatbot\fall_detect_test.csv", new_row) #for hasnain jafri testing
 