import pandas as pd
import numpy as np
from pykalman import KalmanFilter

def kalman_filter(data, transition_covariance, observation_covariance):
    # Initialize Kalman filter
    kf = KalmanFilter(initial_state_mean=data[0], n_dim_obs=1, transition_covariance=transition_covariance, observation_covariance=observation_covariance)
    
    # Apply Kalman filter to smooth the data
    smoothed_data, _ = kf.filter(data)

    return smoothed_data.flatten()

def write_tum_data(path, original_data, smoothed_data):
    with open(path, 'w') as f:
        for i in range(len(original_data)):
            f.write(f"{original_data[i][0]:.10f} ")
            f.write(" ".join([f"{val:.10f}" for val in smoothed_data[i][:2]]))
            f.write(f" {original_data[i][3]:.10f} ")  # Add a space between the fourth and fifth columns
            f.write(" ".join([f"{val:.10f}" for val in original_data[i][4:]]))
            f.write("\n")
        f.close()

def main(input_file, output_file):
    # Load TUM format data
    df = pd.read_csv(input_file, header=None, delimiter=' ', comment='#')

    # Extract the second, third, fourth, and fifth columns
    selected_columns = df.iloc[:, [1, 2, 3, 4, 5, 6, 7]]

    # Apply Kalman filter to each column with adjusted covariance values
    transition_covariance = 0.5  # Adjust as needed
    observation_covariance = 30.0  # Adjust as needed
    smoothed_data = np.apply_along_axis(kalman_filter, axis=0, arr=selected_columns.values, transition_covariance=transition_covariance, observation_covariance=observation_covariance)

    # Save the smoothed results to the output file in TUM format
    write_tum_data(output_file, df.values, smoothed_data)
    print(f"Kalman filtering completed. Results saved to {output_file}")

if __name__ == "__main__":
    input_file = "gt_14.tum"  # Replace with your TUM format file
    output_file = "stamped_groundtruth.txt"  # Replace with the desired output file name

    main(input_file, output_file)

