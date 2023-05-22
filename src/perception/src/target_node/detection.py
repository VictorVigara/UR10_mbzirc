import pyrealsense2 as rs
import cv2
import imutils
import numpy as np

import torch
model = torch.hub.load('src/target_node/yolov5-master', 'custom', path='src/target_node/best.pt', force_reload=True, source = 'local')

def get_center(bbox):
    center = [0,0]
    center[0]=int(bbox[0]+bbox[2]/2)
    center[1]=int(bbox[1]+bbox[3]/2)
    return center

def get_target_pos():
    #Set sensor weights
    w_d_x = 1 
    w_d_y = 1 
    w_d_z = 3

    w_rgb_x = 1
    w_rgb_y = 1 
    w_rgb_z = 1 

    real_area = 0.066

    #####################################################
    ##               Read bag from file                ##
    #####################################################

    file = 'arm_2.bag'

    # Create pipeline
    pipeline = rs.pipeline()

    # Create a config object
    config = rs.config()

    config.enable_stream(rs.stream.color, width=640, height=480)
    config.enable_stream(rs.stream.depth, width=848, height=480)

    # Start streaming from file
    profile = pipeline.start(config)

    # get the depth sensor
    depth_sensor = profile.get_device().first_depth_sensor()

    # get the depth scale
    depth_scale = depth_sensor.get_depth_scale()





    ###########################
    # Get images :

    # Get frameset of depth
    frames = pipeline.wait_for_frames()

    # Get color frames
    color_frame = frames.get_color_frame()

    # Get depth frame
    depth_frame = frames.get_depth_frame()

    depth_image_raw = np.asanyarray(depth_frame.get_data())

    # Convert images to numpy arrays
    color_image = np.asanyarray(color_frame.get_data())
    cv2.imshow('image', color_image)
    
    img_color = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)


    mean_depth = np.mean(depth_image_raw[depth_image_raw<(6/depth_scale)])

    depth_color_image = 1-(depth_image_raw/(mean_depth))

    depth_color_image[depth_color_image > 0.99] = 0

    depth_color_image = 2*depth_color_image


    cv2.imshow("kfr", depth_color_image)
    # Get image's centers
    rgb_center = [int(img_color.shape[1]/2),int(img_color.shape[0]/2)]
    depth_center = [int(depth_color_image.shape[1]/2),int(depth_color_image.shape[0]/2)]

    # get the intrinsics for the RGB and depth streams
    rgb_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
    depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

    rgb_sensor_width = rgb_intrinsics.width
    depth_sensor_width = depth_intrinsics.width

    rgb_sensor_height = rgb_intrinsics.height
    depth_sensor_height = depth_intrinsics.height

    # convert the intrinsic matrices to numpy arrays
    K_rgb = np.array([[rgb_intrinsics.fx, 0, rgb_intrinsics.ppx],
                   [0, rgb_intrinsics.fy, rgb_intrinsics.ppy],
                   [0, 0, 1]])

    rgb_focal_length_x = rgb_intrinsics.fx
    rgb_focal_length_y = rgb_intrinsics.fy

    K_rgb_inv = np.linalg.inv(K_rgb)

    K_depth = np.array([[depth_intrinsics.fx, 0, depth_intrinsics.ppx],
                   [0, depth_intrinsics.fy, depth_intrinsics.ppy],
                   [0, 0, 1]])

    depth_focal_length_x = depth_intrinsics.fx
    depth_focal_length_y = depth_intrinsics.fy

    # Calculate the pixel size
    rgb_x_pixel_size = rgb_sensor_width / img_color.shape[1]
    rgb_y_pixel_size = rgb_sensor_height / img_color.shape[0]

    depth_x_pixel_size = depth_sensor_width / depth_color_image.shape[1]
    depth_y_pixel_size = depth_sensor_height / depth_color_image.shape[0]






    ##########################
    # RGB detection
    # YOLO

    results = model(img_color)
    size = int(results.pandas().xyxy[0].size/7)
    i = 0
    if size != 0:
        x1 = results.xyxy[0][i][0].cpu().numpy()
        y1 = results.xyxy[0][i][1].cpu().numpy()
        x2 = results.xyxy[0][i][2].cpu().numpy()
        y2 = results.xyxy[0][i][3].cpu().numpy()

        w = int(abs(x1-x2))
        h = int(abs(y1-y2))
        x = int(x1)
        y = int(y1)
    bbox = x, y, w, h

#     bbox = 333, 199, 125, 88 #646, 206, 192, 161

    # bbox = cv2.selectROI('Initialisation',img_color, False)
    # cv2.destroyWindow('Initialisation')
    # # print(bbox)
#     x, y, w, h = bbox


    # Visualize
    yolo_img = img_color.copy()
    center = get_center(bbox)
    cv2.circle(yolo_img, (center[0],center[1]), radius=3, color=(0, 0, 255), thickness=-1)
    cv2.rectangle(yolo_img,(x,y),(x+w,y+h),(0,0,255),2)
    cv2.imshow('YOLO', yolo_img)

    # Compute x y z from rgb
    area = w*h
    x_pixel = center[0] - rgb_center[0]
    y_pixel = rgb_center[1] - center[1]

    distance_rgb_z = ((real_area * rgb_focal_length_x* rgb_focal_length_y) / area) ** 0.5

    distance_rgb_x = (distance_rgb_z * x_pixel * rgb_x_pixel_size) / rgb_focal_length_x
    distance_rgb_y = (distance_rgb_z * y_pixel * rgb_y_pixel_size) / rgb_focal_length_y





    #################
    # Depth detection
    # Transform RGB detection to depth frame
    point_3d_rgb_1 = np.dot(K_rgb_inv, np.array([x, y, 1]))
    point_3d_rgb_2 = np.dot(K_rgb_inv, np.array([x + w, y + h, 1]))
    point_2d_depth_1 = np.dot(K_depth, point_3d_rgb_1)
    point_2d_depth_1[0] = point_2d_depth_1[0] - 25
    point_2d_depth_1[1] = point_2d_depth_1[1] - 20
    point_2d_depth_2 = np.dot(K_depth, point_3d_rgb_2)
    point_2d_depth_2[0] = point_2d_depth_2[0] + 65
    point_2d_depth_2[1] = point_2d_depth_2[1] + 20

    cv2.rectangle(depth_color_image, (int(point_2d_depth_1[0]), int(point_2d_depth_1[1])),
         (int(point_2d_depth_2[0]), int(point_2d_depth_2[1])), (0, 0, 0), 2)

    # Crop depth image using RGB detection
    mask = np.zeros(depth_color_image.shape[:2], dtype=np.uint8)
    cv2.rectangle(mask, (int(point_2d_depth_1[0]), int(point_2d_depth_1[1])),
            (int(point_2d_depth_2[0]), int(point_2d_depth_2[1])), 1, -1)
    crop = cv2.bitwise_and(depth_color_image.copy(), depth_color_image.copy(), mask=mask)


    # Process depth image
    blur = cv2.blur(crop, (19, 19))

    max_val = blur.max()

    thresh = cv2.threshold(blur, max_val*(200/255), 255, cv2.THRESH_BINARY)[1]

    kernel = np.ones((19, 19), np.uint8)

    dilate = cv2.dilate(thresh, kernel, iterations=1)


    # Find contours of the box
    cnts = cv2.findContours(dilate.copy().astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # loop over the contours
    for cs in cnts:
        # Check area
        area = cv2.contourArea(cs)
        if area < 4000:
            break
        # compute the center of the contour
        M = cv2.moments(cs)
        if M["m00"] > 0 :
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # draw the contour and center of the shape on the image
            cv2.drawContours(depth_color_image, [cs], -1, (1, 1, 1), 2)
            cv2.circle(depth_color_image, (cX, cY), 5, (0, 0, 0), -1)

            #Get distance
            depth_z_dist = depth_image_raw[cY, cX] * depth_scale

            depth_x_pixel = cX - depth_center[0]
            depth_y_pixel = depth_center[1] - cY

            depth_x_dist = (depth_z_dist * depth_x_pixel * depth_x_pixel_size) / depth_focal_length_x
            depth_y_dist = (depth_z_dist * depth_y_pixel * depth_y_pixel_size) / depth_focal_length_y
            break

    cv2.imshow('Depth detection', depth_color_image)        


    ################
    # Position estimation
    # Combine measurements
    z = [0,0,0]
    z[0] = (distance_rgb_x * w_rgb_x + depth_x_dist * w_d_x) / (w_rgb_x + w_d_x)
    z[1] = (distance_rgb_y * w_rgb_y + depth_y_dist * w_d_y) / (w_rgb_y + w_d_y)
    z[2] = (distance_rgb_z * w_rgb_z + depth_z_dist * w_d_z) / (w_rgb_z + w_d_z)
    print(z)
    
    pipeline.stop()
    cv2.waitKey(500)
    cv2.destroyAllWindows()

    return z




###############
# MAIN
###############
#pos = get_target_pos()
#print(pos)
