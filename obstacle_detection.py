#!/usr/bin/env python3
import rclpy
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
import time
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import queue
from scipy.spatial import cKDTree
# utility function to publish a point cloud ROS message
def publish_cloud(cloud, timestamp, colors=None):
    global pcMsg, pubCloud
    pcMsg.header.stamp = timestamp
    if colors is None:
        pcMsg.fields = [
            PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
        ]
        pcMsg.point_step = 12
    else:
        pcMsg.fields = [
            PointField(name='x',offset=0,datatype=PointField.FLOAT32,count=1),
            PointField(name='y',offset=4,datatype=PointField.FLOAT32,count=1),
            PointField(name='z',offset=8,datatype=PointField.FLOAT32,count=1),
            PointField(name='rgb',offset=12,datatype=PointField.INT32,count=1),
        ]
        pcMsg.point_step = 16
    pcMsg.width = len(cloud)
    pcMsg.row_step = pcMsg.width * pcMsg.point_step
    if colors is None:
        pcMsg.data = cloud.tobytes()
    else:
        color_int = colors[:,0] << 16 | colors[:,1] << 8 | colors[:,2]
        combined = np.rec.fromarrays([cloud[:,0], cloud[:,1], cloud[:,2], color_int], names='x,y,z,c')
        pcMsg.data = combined.tobytes()
    pubCloud.publish(pcMsg)

# utility function to publish bounding boxes as a ROS message
def publish_boxes(boxes, timestamp):
    global boxMsg, pubBoxes
    boxMsg.header.stamp = timestamp
    boxMsg.points = []
    for box in boxes:
        p1 = Point(x=box[0,0], y=box[0,1], z=box[0,2])
        p2 = Point(x=box[1,0], y=box[1,1], z=box[1,2])
        p3 = Point(x=box[2,0], y=box[2,1], z=box[2,2])
        p4 = Point(x=box[3,0], y=box[3,1], z=box[3,2])
        p5 = Point(x=box[4,0], y=box[4,1], z=box[4,2])
        p6 = Point(x=box[5,0], y=box[5,1], z=box[5,2])
        p7 = Point(x=box[6,0], y=box[6,1], z=box[6,2])
        p8 = Point(x=box[7,0], y=box[7,1], z=box[7,2])
        boxMsg.points.extend([p1,p2,p1,p3,p2,p4,p3,p4,p1,p5,p2,p6,p3,p7,p4,p8,p5,p6,p5,p7,p6,p8,p7,p8])
    pubBoxes.publish(boxMsg)

# TODO: implementation function to filter out points with Z value below a specified threshold
def filter_ground(cloud, ground_level=1.0):


    # Filter out points where the Z-coordinate is below the ground level
    filtered_cloud = cloud[cloud[:, 2] > ground_level]

    return filtered_cloud
    
# TODO: implementation function to filter out points further than a specified distance
def filter_by_distance(cloud, distance_threshold=10.0):
    # Calculate the horizontal distance (sqrt(x^2 + y^2)) for each point
    #https://www.expii.com/t/distance-formula-4560
    horizontal_distances = np.sqrt(cloud[:, 0]**2 + cloud[:, 1]**2)
    
    # Filter out points where the horizontal distance is greater than the distance threshold
    filtered_cloud = cloud[horizontal_distances <= distance_threshold]

    return filtered_cloud

# TODO: implementation function to perform Euclidean clustering at a specified threshold in meters
def euclidean_clustering(cloud, threshold=0.5):

    cluster_labels = np.zeros(len(cloud), dtype=int)  
    tree = cKDTree(cloud)  # Build a KD-Tree for efficient neighbor searching
    # I had tried making this without the KD tree and just the que but it was to slow/ didn't work
    #I looked up expamples and some used a kd-tree so that why I used this
    # so chat gpt helped to implement the kdtree which for me was quicker for some reason
    cluster_id = 0  # Start cluster counter
    
    for i in range(len(cloud)):
        if cluster_labels[i] != 0:  # Skip if already labeled
            continue
        
        # Start a new cluster
        cluster_id += 1
        q = queue.Queue()  #
        q.put(i)  
        cluster_labels[i] = cluster_id  
        
        while not q.empty():
            point_idx = q.get()  
            neighbors = tree.query_ball_point(cloud[point_idx], threshold)  
            
            for neighbor in neighbors:
                if cluster_labels[neighbor] == 0:  #
                    q.put(neighbor)  
                    cluster_labels[neighbor] = cluster_id  
    
    return cluster_labels
# TODO: (extra credit) implementation function to perform Euclidean clustering with lower computation time
def euclidean_clustering_accelerated(cloud, threshold=0.5):
    cluster_labels = np.zeros(len(cloud), dtype=int)
    return cluster_labels

# TODO: (extra credit) implementation function to filter clusters by number of points
def filter_clusters(cluster_labels, min_num_points=100):
    return cluster_labels

# TODO: implementation function to compute bounding boxes from cluster labels
def get_bounding_boxes(cloud, cluster_labels):
    """
    Computes the 3D bounding boxes for each cluster in the point cloud.
    
    Args:
    cloud (numpy.ndarray): The input point cloud as a (N, 3) array.
    cluster_labels (numpy.ndarray): An array of cluster labels for each point in the cloud.
    
    Returns:
    list: A list of 3D bounding boxes represented as 8 corner points in the format:
          [ [x1, y1, z1], [x2, y2, z2], ..., [x8, y8, z8] ]
    """
    unique_labels = np.unique(cluster_labels)
    boxes = []

    for label in unique_labels:
        if label == 0:
            continue  # Skip unclustered points (label 0)

        # Get points belonging to the current cluster
        cluster_points = cloud[cluster_labels == label]

        # Calculate the minimum and maximum coordinates for the bounding box
        min_coords = np.min(cluster_points, axis=0)
        max_coords = np.max(cluster_points, axis=0)

        # Define the 8 corners of the 3D bounding box
        box = np.array([
            [min_coords[0], min_coords[1], min_coords[2]],  # Bottom-front-left
            [max_coords[0], min_coords[1], min_coords[2]],  # Bottom-front-right
            [min_coords[0], max_coords[1], min_coords[2]],  # Bottom-back-left
            [max_coords[0], max_coords[1], min_coords[2]],  # Bottom-back-right
            [min_coords[0], min_coords[1], max_coords[2]],  # Top-front-left
            [max_coords[0], min_coords[1], max_coords[2]],  # Top-front-right
            [min_coords[0], max_coords[1], max_coords[2]],  # Top-back-left
            [max_coords[0], max_coords[1], max_coords[2]]   # Top-back-right
        ], dtype=np.float64)

        # Add the bounding box to the list
        boxes.append(box)

    return boxes

# callback function to subscribe to the ROS point cloud message input
def point_cloud_callback(msg):
    global node, count_msg
    start_time = time.time()
    input_cloud = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)

    # Extract only the X, Y, Z columns (ignore the intensity column)
    input_cloud = input_cloud[:, :3]

    # Filter out ground points
    filtered_cloud = filter_ground(input_cloud, -1.2)

    # Filter out points that are out of range
    filtered_cloud = filter_by_distance(filtered_cloud, 10)

    # Determine cluster labels
    cluster_labels = euclidean_clustering(filtered_cloud[:, :3])

    # Filter out clusters that are too small
    cluster_labels = filter_clusters(cluster_labels, 100)

    # Set a unique color for each cluster
    colors = np.zeros((len(filtered_cloud), 3), dtype=np.int32)
    for i in range(cluster_labels.min(), cluster_labels.max()+1):
        cluster_mask = cluster_labels == i
        colors[cluster_mask] = np.random.randint(0,255,3)

    # Compute a bounding box for each cluster
    boxes = get_bounding_boxes(filtered_cloud, cluster_labels)
    publish_boxes(boxes, msg.header.stamp)

    publish_cloud(filtered_cloud[:, :3], msg.header.stamp, colors)
    count_msg += 1
    end_time = time.time()
    node.get_logger().info("Message %d: Processed %d points %d clusters in %.3fs" % (count_msg, len(input_cloud), len(set(cluster_labels) - set([0])), end_time - start_time))

def main():
    rclpy.init()
    global node, count_msg, pcMsg, pubCloud, boxMsg, pubBoxes
    node = rclpy.create_node('obstacle_detection')
    pubCloud = node.create_publisher(PointCloud2, 'output_cloud', 1)
    pubBoxes = node.create_publisher(Marker, 'boxes', 1)
    scan_subscription = node.create_subscription(PointCloud2, 'kitti/velo/pointcloud', point_cloud_callback, 1)

    # initialize the output ROS messages
    count_msg = 0
    pcMsg = PointCloud2()
    pcMsg.header.frame_id = 'velo_link'
    pcMsg.height = 1
    pcMsg.is_bigendian = False
    pcMsg.is_dense = True

    boxMsg = Marker()
    boxMsg.header.frame_id = "velo_link"
    boxMsg.type = Marker.LINE_LIST
    boxMsg.lifetime = rclpy.time.Duration().to_msg()
    boxMsg.color.a = 1.0
    boxMsg.action = Marker.ADD
    boxMsg.scale.x = 0.05
    boxMsg.pose.orientation.w = 1.0
    boxMsg.id = 0
    boxMsg.color.r = 1.0
    boxMsg.color.g = 1.0
    boxMsg.color.b = 0.0

    rclpy.spin(node)

if __name__ == '__main__':
    main()
