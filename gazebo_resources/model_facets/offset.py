import csv
import numpy as np

def compute_offset_points(centers, normals, offset_distance):

    # Ensure the normals are unit vectors
    normalized_normals = normals / np.linalg.norm(normals, axis=1)[:, None]
    # Compute the offset points
    offset_points = centers + offset_distance * normalized_normals
    return offset_points

def read_csv(file_path):
  
    centers = []
    normals = []
    with open(file_path, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        for row in reader:
            centers.append([float(row[0]), float(row[1]), float(row[2])])
            normals.append([float(row[3]), float(row[4]), 0.0 ])
    return np.array(centers), np.array(normals)

def write_csv(file_path, centers, normals, offset_points):
  
    with open(file_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile, delimiter=',')
        writer.writerow(['x', 'y', 'z', 'nx', 'ny', 'nz', 'offset_x', 'offset_y', 'offset_z'])
        for i in range(len(centers)):
            writer.writerow([centers[i,0], centers[i,1], centers[i,2],
                             normals[i,0], normals[i,1], normals[i,2],
                             offset_points[i,0], offset_points[i,1], offset_points[i,2]])

# Input and output CSV file paths
input_csv_path = 'boat.csv'  # Replace with your input file path
output_csv_path = 'output_0-75n.csv' # Replace with your desired output file path

# Read data from the existing CSV file
centers, normals = read_csv(input_csv_path)

# Compute offset points
offset_distance = 0.75  # Offset by 1 meter
offset_points = compute_offset_points(centers, normals, offset_distance)

# Write to the new CSV file
write_csv(output_csv_path, centers, normals, offset_points)

