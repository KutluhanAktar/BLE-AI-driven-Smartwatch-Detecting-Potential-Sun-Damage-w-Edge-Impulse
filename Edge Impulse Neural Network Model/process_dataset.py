# BLE AI-driven Smartwatch Detecting Potential Sun Damage w/ Edge Impulse
#
# Windows, Linux, or Ubuntu
#
# By Kutluhan Aktar
#
# Log UV & weather data on an SD card to train an Edge Impulse model. Then, run it to get informed of sun damage over BLE via an Android app.
# 
#
# For more information:
# https://www.theamplituhedron.com/projects/BLE_AI_driven_Smartwatch_Detecting_Potential_Sun_Damage_w_Edge_Impulse

import numpy as np
import pandas as pd
from csv import writer

# Create a class to modify the given data set so as to upload properly formatted samples to Edge Impulse.
class process_dataset:
    def __init__(self, csv_path):
        # Read the data set from the given CSV file.
        self.df = pd.read_csv(csv_path)
        # Define the class (label) names.
        self.class_names = ["Tolerable", "Risky", "Perilous"]
    # Scale (normalize) data to define appropriately formatted inputs.
    def scale_data_elements(self):
        self.df["scaled_uv_index"] = self.df["uv_index"] / 10
        self.df["scaled_temperature"] = self.df["temperature"] / 100
        self.df["scaled_pressure"] = self.df["pressure"] / 100000
        self.df["scaled_altitude"] = self.df["altitude"] / 100
        print("Data Elements Scaled Successfully!")
    # Split the data set to generate a separate CSV file for each sample.     
    def split_dataset_by_labels(self, class_number):
        l = len(self.df)
        sample_number = 0
        # Split the data set according to sun damage risk levels (classes):
        for i in range(l):
            # Add the header as the first row:
            processed_data = [["uv_index","temperature","pressure","altitude"]]
            if (self.df["risk_level"][i] == class_number):
                row = [self.df["scaled_uv_index"][i], self.df["scaled_temperature"][i], self.df["scaled_pressure"][i], self.df["scaled_altitude"][i]]
                processed_data.append(row)
                # Increase the sample number for each sample:   
                sample_number+=1   
                # Create a CSV file for each sample identified with the sample number.
                filename = "{}.sample_{}.csv".format(self.class_names[class_number], sample_number)
                with open(filename, "a", newline="") as f:
                    for r in range(len(processed_data)):
                        writer(f).writerow(processed_data[r])
                    f.close()
                print("CSV File Successfully Created: " + filename)
        
# Define a new class object named 'dataset':
dataset = process_dataset("UV_DATA.CSV")

# Scale data and generate a separate CSV file for each sample:
dataset.scale_data_elements()
for c in range(len(dataset.class_names)):
    dataset.split_dataset_by_labels(c)
            