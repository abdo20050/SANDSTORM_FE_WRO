import cv2
import numpy as np
from picamera2 import Picamera2
import json
import os
import time

# HSV ranges for the colors
color_ranges = {
    'black': {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 50])},
    'orange': {'lower': np.array([5, 100, 100]), 'upper': np.array([15, 255, 255])},
    'blue': {'lower': np.array([100, 150, 0]), 'upper': np.array([140, 255, 255])},
    'green': {'lower': np.array([40, 100, 100]), 'upper': np.array([70, 255, 255])},
    'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
    'maginta': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])}
}
# File to store the HSV values
json_file_path = '/home/jojo/rc_run_v2/hsv_values.json'

# Function to load HSV values from the JSON file (if it exists)
def load_hsv_from_json():
    global color_ranges
    if os.path.exists(json_file_path):
        with open(json_file_path, 'r') as json_file:
            saved_data = json.load(json_file)
        
        # Load saved data into the color_ranges
        for mask in saved_data:
            color_ranges[mask]['lower'] = np.array(saved_data[mask]['lower'])
            color_ranges[mask]['upper'] = np.array(saved_data[mask]['upper'])
        print("json loaded!")
    else:
        print("no json file detected!")
# Minimum contour area threshold to filter small contours
MIN_CONTOUR_AREA = 10  # You can adjust this value as needed

# Function to detect the color contours and return bounding rectangles
def detect_color_contours(hsv_frame, color):
    mask = cv2.inRange(hsv_frame, color_ranges[color]['lower'], color_ranges[color]['upper'])
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)
    # Apply thresholding after the blur
    _, thresh_mask = cv2.threshold(blurred_mask, 127, 255, cv2.THRESH_BINARY)
    contours, _ = cv2.findContours(thresh_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours by area
    filtered_contours = [c for c in contours if cv2.contourArea(c) > MIN_CONTOUR_AREA]
    
    return filtered_contours

def display_live_window(image, window_name="Live Tracking"):
    """Utility function to display images in OpenCV window."""
    cv2.imshow(window_name, image)
    cv2.waitKey(1)  # 1 ms wait to allow OpenCV to process the window

def find_bottommost_contour(contours, image_height):
    """
    Find the contour closest to the bottom of the image.

    :param contours: List of contours.
    :param image_height: Height of the image.
    :return: The contour closest to the bottom.
    """
    max_bottom = 0
    bottommost_contour = None

    for contour in contours:
        # Find the bottommost point of the contour
        bottom_point = max(contour, key=lambda point: point[0][1])
        bottom_y = bottom_point[0][1]
        
        # Update the closest contour if this one is lower
        if bottom_y > max_bottom:
            max_bottom = bottom_y
            bottommost_contour = contour

    return bottommost_contour
def generate_path(filtered_track):
    """
    Generate a path based on the centerline of the filtered track.

    :param filtered_track: Binary image with the track in white pixels.
    :return: Path points [(x1, y1), (x2, y2), ...].
    """
    height, width = filtered_track.shape
    path_points = []

    for y in range(height):
        # Find white pixels in the current row
        white_pixels = np.where(filtered_track[y, :] == 255)[0]
        if len(white_pixels) > 0:
            # Calculate the midpoint of the white pixels
            center_x = (white_pixels[0] + white_pixels[-1]) // 2
            path_points.append((center_x, y))

    return np.array(path_points)

def filter_path_points(path_points, smoothing_window=5, deviation_threshold=20):
    """
    Filter noisy path points using smoothing and deviation filtering.

    :param path_points: Array of (x, y) path points.
    :param smoothing_window: Size of the moving average smoothing window.
    :param deviation_threshold: Maximum allowable deviation from the fitted path.
    :return: Filtered path points.
    """
    np.array(path_points)
    # Separate x and y coordinates
    x_values = path_points[:, 0]
    y_values = path_points[:, 1]

    # Step 1: Moving Average Smoothing
    smooth_x_values = np.convolve(
        x_values, np.ones(smoothing_window) / smoothing_window, mode="same"
    )

    # Step 2: Fit a polynomial to the smoothed points
    fit = np.polyfit(y_values, smooth_x_values, deg=2)  # Quadratic fit
    fitted_x_values = np.polyval(fit, y_values)

    # Step 3: Remove points with large deviations
    deviations = np.abs(x_values - fitted_x_values)
    filtered_indices = deviations < deviation_threshold
    filtered_path_points = path_points[filtered_indices]

    return filtered_path_points

# Initialize persistent variables
last_contour_time = 0
persistent_left_region = None
persistent_right_region = None

def process_frame(frame, num_points_avg):
    global last_contour_time, persistent_left_region, persistent_right_region

    original = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB for plt
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Step 2: Filter Black Walls and Highlight White Track
    gray = cv2.GaussianBlur(gray, ksize=(5,5), sigmaX=10)
    _, white_mask = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)  # White track // change the threshold 
    _, black_mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # Black walls
    height, width = gray.shape

    hsv_frame = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)
    green_mask = np.zeros_like(original)
    green_contours = detect_color_contours(hsv_frame, 'green')
    cv2.drawContours(green_mask, green_contours, -1, (0,255,0), thickness=cv2.FILLED)
    
    red_mask = np.zeros_like(original)
    red_contours = detect_color_contours(hsv_frame, 'red')
    cv2.drawContours(red_mask, red_contours, -1, (0,0,255), thickness=cv2.FILLED)
    inverted_black_mask = 255-np.array(black_mask)
    # contours, _ = cv2.findContours(white_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours, _ = cv2.findContours(inverted_black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour_index = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
    filtered_track = np.zeros_like(gray)
    cv2.drawContours(filtered_track, contours, largest_contour_index, 255, thickness=cv2.FILLED)
    # Get the middle column of the image
    middle_x = width // 2
    middle_column = filtered_track[:, middle_x]

    # Find indices where the pixel value is white (255)
    white_indices = np.where(middle_column == 255)[0]
    wall_point = None
    if white_indices.size > 0:
        # Get the lowest y-value (maximum index since y increases downward in images)
        lowest_y = white_indices[0]
        wall_point = (middle_x, lowest_y)
        # print(f"Lowest black point in the middle column: {wall_point}")
    else:
        print("No black pixels found in the middle column.")

    roi = np.zeros_like(gray)
    left_region = 10
    right_region = width - 10
    lower_region = int(height * 0.3)  # Adjust the starting point of the lower ROI
    higher_region = int(height * 0.7)
    flag = ""

    contours_detected = False
    red_detected = False
    green_detected = False
    obstical_base_point = [0,0]
    def pass_red(red_contours):
        nonlocal left_region, right_region, height, original , contours_detected, obstical_base_point, red_detected      
        for contour in red_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(original, (x, y), (x + w, y + h), (0,0,255), 2)
            # print(x,y,w,h)
            if y+h > int(height*0.4):
                contours_detected = True
                red_detected = True
                if y+h > obstical_base_point[1]:
                    obstical_base_point[1] = y+h
                if (x+w) > left_region:
                    left_region = (x+w) + 20 #pixels old value: 30
                    obstical_base_point[0] = left_region
    def pass_green(green_contours):
        nonlocal left_region, right_region, height, original, contours_detected, obstical_base_point,green_detected
        for contour in green_contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(original, (x, y), (x + w, y + h), (0,255,0), 2)
            # print(x,y,w,h)
            if y+h > int(height*0.4):
                contours_detected = True
                green_detected = True
                if y+h > obstical_base_point[1]:
                    obstical_base_point[1] = y+h
                if x < right_region:
                    right_region = x - 20
                    obstical_base_point[0] = right_region

    if green_contours and red_contours:
        h_red = max([cv2.boundingRect(contour)[3] for contour in red_contours])
        h_green = max([cv2.boundingRect(contour)[3] for contour in green_contours])
        if h_red > h_green:
            pass_red(red_contours)
        else:
            pass_green(green_contours)

    elif green_contours:
        pass_green(green_contours)      

    elif red_contours:
        pass_red(red_contours)
    
    # Logic to hold regions for 2 seconds after contours disappear
    if contours_detected:
        higher_region = height
        # Update persistent regions and timestamp
        persistent_left_region = left_region
        persistent_right_region = right_region
        last_contour_time = time.time()
    else:
        # Check if contours disappeared less than 2 seconds ago
        if time.time() - last_contour_time < 0.9: # 0.9 is the time to keep the track after obstacle passed
            left_region = persistent_left_region if persistent_left_region is not None else left_region
            right_region = persistent_right_region if persistent_right_region is not None else right_region
    pre_roi = np.zeros_like(roi)
    roi[lower_region:higher_region, left_region:right_region] = 255  # Keep only the lower region
    
    if red_detected:    
        for y in range(obstical_base_point[1],height):
            # Calculate the x position for the slope
            slope = (obstical_base_point[1]-height)/(obstical_base_point[0]-width//2 + 0.01)
            b = obstical_base_point[1] + obstical_base_point[0]*slope
            x = int((y-b)/slope)
            pre_roi[y, x-10:min(width, x + 40)] = 255  # Set the left edge to black in the mask

        
        roi[obstical_base_point[1]: , :] = pre_roi[obstical_base_point[1]: , :]
    elif green_detected:
        for y in range(obstical_base_point[1],height):
            # Calculate the x position for the slope
            slope = (obstical_base_point[1]-height)/(obstical_base_point[0]-width//2 + 0.01)
            b = obstical_base_point[1] + obstical_base_point[0]*slope
            x = int((y-b)/slope)
            pre_roi[y, max(0,x-40):x+10] = 255  # Set the left edge to black in the mask

        
        roi[obstical_base_point[1]: , :] = pre_roi[obstical_base_point[1]: , :]
    filtered_track = cv2.bitwise_and(filtered_track, roi)
   
    
    path_points = generate_path(filtered_track)
    try:
        path_points = filter_path_points(path_points, smoothing_window=10) 
    except:
        pass

    for i in range(len(path_points) - 1):
        cv2.line(original, path_points[i], path_points[i + 1], (255, 0, 0), 2)
    # start_path_point = max(path_points[:,1])
    # print(start_path_point)
    # print(path_points[-4:])


    try:
        offset_steer =  width//2 - int(np.mean(path_points[-num_points_avg: ,0]))
        # if start_path_point > height//2:
        #     # offset_steer = path_points[path_points[:,1]==77][0][0] - path_points[path_points[:,1]==50][0][0]
        # else:
        #     offset_steer = 0
    except:
        offset_steer = 0
    # print(offset_steer)

    cv2.line(original, 
            (width//2, height), 
            (width//2 - offset_steer, height*3//4), 
            (0,255,0), 2)  # Path line
    
    filtered_track_rgb = cv2.cvtColor(filtered_track, cv2.COLOR_GRAY2RGB, red_mask)
    # Display all images in OpenCV window
    combined_image = np.hstack([original, filtered_track_rgb])
    # display_live_window(combined_image)
    return offset_steer, wall_point, combined_image, flag

# Function to detect and draw contours on the original frame
def draw_contours_on_original(original_frame, mask):
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours on the original frame
    contour_frame = original_frame.copy()
    cv2.drawContours(contour_frame, contours, -1, (0, 255, 0), 2)  # Green contours with thickness of 2
    
    return contour_frame


if __name__ == "__main__":
# Step 1: Capture a frame from Picamera2
    # Initialize Picamera2
    picam2 = Picamera2()
    RESIZE_FACTOR = 1
    # picam2.sensor_mode = 4
    #2304 x 1296-PC1B
    cam_res = (2304 // RESIZE_FACTOR, 1296 // RESIZE_FACTOR)

    video_config = picam2.create_video_configuration(raw={"size": cam_res, "format" : 'SGBRG8'}, main = {"size":(2304//10, 1296//10)})
    picam2.configure(video_config)
    picam2.start()

    load_hsv_from_json()
    # print(color_ranges)
    while True:

        frame = picam2.capture_array()  # Capture the frame from Picamera2
        # Step 1: Verify the frame shape
    
       
        offset_steer, wall_point, comp_img, flag = process_frame(frame, 40)
        
        # # frame_bgr = frame
        # original = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # Convert to RGB for plt
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # hsv_frame = cv2.cvtColor(original, cv2.COLOR_BGR2HSV)

        
        # # cv2.imshow('Calibration', combined_frame)
        # # if cv2.waitKey(1) & 0xFF == ord('q'):
        # #     break
        # # continue
        # # print(hsv_frame)
        # # red_mask = np.zeros_like(original)
        # # red_contours = detect_color_contours(hsv_frame, 'red')
        # # cv2.drawContours(red_mask, red_contours, -1, (0,0,255), 2)
        
        # green_mask = np.zeros_like(original)
        # green_contours = detect_color_contours(hsv_frame, 'green')
        # cv2.drawContours(green_mask, green_contours, -1, (0,255,0), thickness=cv2.FILLED)
        
        
        # # Step 2: Filter Black Walls and Highlight White Track
        # gray = cv2.GaussianBlur(gray, ksize=(5,5), sigmaX=10)
        # _, white_mask = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)  # White track
        # _, black_mask = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV)  # Black walls

        # # Step 3: Detect Edges
        # edges = cv2.Canny(black_mask, 50, 150)

        # # Define a lower region of interest (ROI)
        # height, width = edges.shape
        # roi = np.zeros_like(edges)
        # lower_region = int(height * 0.3)  # Adjust the starting point of the lower ROI
        # higher_region = int(height * 0.5)
        # roi[lower_region:higher_region, :] = 255  # Keep only the lower region

        # # Apply the ROI mask
        # masked_edges = cv2.bitwise_and(edges, roi)
        # # Detect line segments using Hough Line Transform
        # lines = cv2.HoughLinesP(masked_edges, rho=2, theta=np.pi / 180, threshold=30, minLineLength=0, maxLineGap=30)

            
        # # Initialize variables for left and right points
        # left_points = []
        # right_points = []
        # if lines is not None:
        #     # Classify lines into left and right based on x-coordinate
        #     for line in lines:
        #         x1, y1, x2, y2 = line[0]
        #         midpoint_x = (x1 + x2) / 2
        #         if midpoint_x < width / 2:  # Left side
        #             left_points.append((x1, y1))
        #             left_points.append((x2, y2))
        #         else:  # Right side
        #             right_points.append((x1, y1))
        #             right_points.append((x2, y2))

        # # Fit linear lines to the left and right points
        # # Draw the left, right, and path lines
        # line_image = np.zeros_like(edges)
        # offset_steer = 0
        # # if left_points or right_points:
        # #     # Generate y-values for the path line
        # #     y_values = np.linspace(lower_region, height//2, num=50)
        # #     if right_points:
        # #         right_points = np.array(right_points)
        # #         right_fit = np.polyfit(right_points[:, 1], right_points[:, 0], 2)  # Fit x as a function of y
        # #         right_x_values = np.polyval(right_fit, y_values)

        # #     if left_points:
        # #         left_points = np.array(left_points)
                
        # #         # Linear fit: y = mx + b -> x = (y - b) / m
        # #         left_fit = np.polyfit(left_points[:, 1], left_points[:, 0], 2)  # Fit x as a function of y
                

        # #         # Calculate x-values for the left and right polynomial fits
        # #         left_x_values = np.polyval(left_fit, y_values)

        # #     # Calculate path points as midpoints
        # #     if len(right_points) and not len(left_points):
        # #         path_x_values = right_x_values - width// 2
        # #         path_x_values //=5
        # #     elif len(left_points) and not len(right_points):
        # #         path_x_values = left_x_values + width//2
        # #         path_x_values //=5
        # #     else:
        # #         path_x_values = (right_x_values + left_x_values) // 2
            
        # #     path_points = np.array(list(zip(path_x_values, y_values)), dtype=np.int32)
        # #     # print(path_points[path_points[:,1].argmax()][0])
        # #     offset_steer = min(20, max(-20 ,int(path_points[path_points[:,1].argmax()][0]) - int(path_points[path_points[:,1].argmin()][0])))

        # #     # Draw the left, right, and path lines
        # #     line_image = np.zeros_like(edges)
        # #     for i in range(len(y_values) - 1):
        # #         # cv2.line(line_image, 
        # #         #         (int(left_x_values[i]), int(y_values[i])), 
        # #         #         (int(left_x_values[i+1]), int(y_values[i+1])), 
        # #         #         255, 2)  # Left line
        # #         # cv2.line(line_image, 
        # #         #         (int(right_x_values[i]), int(y_values[i])), 
        # #         #         (int(right_x_values[i+1]), int(y_values[i+1])), 
        # #         #         255, 2)  # Right line
        # #         cv2.line(line_image, 
        # #                 (int(path_x_values[i]), int(y_values[i])), 
        # #                 (int(path_x_values[i+1]), int(y_values[i+1])), 
        # #                 255, 2)  # Path line
                
        
        # # cv2.line(original, 
        # #                 (width//2, height), 
        # #                 (width//2 - offset_steer, height*3//4), 
        # #                 (0,255,0), 2)  # Path line
        

        # # Step 4: Filter the White Track
        # inverted_black_mask = 255-np.array(black_mask)
        # contours, _ = cv2.findContours(inverted_black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # # contour = find_bottommost_contour(contours, height)
        # largest_contour_index = max(range(len(contours)), key=lambda i: cv2.contourArea(contours[i]))
        # filtered_track = np.zeros_like(gray)
        # cv2.drawContours(filtered_track, contours, largest_contour_index, 255, thickness=cv2.FILLED)
        # roi = np.zeros_like(edges)
        # left_region = 0
        # right_region = width
        # if green_contours:
            
        #     max_contour = max(contours, key=cv2.contourArea)
        #     for contour in green_contours:
        #         x, y, w, h = cv2.boundingRect(contour)
        #         cv2.rectangle(original, (x, y), (x + w, y + h), (0,255,0), 2)
        #         # print(x,y,w,h)
        #         if x < right_region:
        #             right_region = x
        #     # print(right_region)
        # lower_region = int(height * 0.3)  # Adjust the starting point of the lower ROI
        # higher_region = int(height * 0.7)
        
        # roi[lower_region:higher_region, left_region:right_region] = 255  # Keep only the lower region
        
        
        # filtered_track = cv2.bitwise_and(filtered_track, roi)
        # # Iterate through each column
        # for x in range(width):
        #     # Flag to indicate if a black pixel has been encountered in the column
        #     black_found = False
        #     for y in range(int(height*0.2),height):
        #         if filtered_track[y, x] == 0:  # Black pixel encountered
        #             black_found = True
        #             break
        #         else:
        #             # Black out the current pixel in the white mask
        #             filtered_track[y, x] = 0    
        
        # path_points = generate_path(filtered_track)
        # try:
        #     path_points = filter_path_points(path_points, smoothing_window=10) 
        # except:
        #     pass
        # for i in range(len(path_points) - 1):
        #     cv2.line(original, path_points[i], path_points[i + 1], (255, 0, 0), 2)
        # try:
        #     offset_steer = path_points[path_points[:,1]==77][0][0] - path_points[path_points[:,1]==50][0][0]
        # except:
        #     offset_steer = 0
        # # print(offset_steer)
        # cv2.line(original, 
        #                 (width//2, height), 
        #                 (width//2 - offset_steer, height*3//4), 
        #                 (0,255,0), 2)  # Path line
        # # Step 5: Find the Path (Centroids)
        # moments = [cv2.moments(cnt) for cnt in contours if cv2.contourArea(cnt) > 500]
        # centroids = [(int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])) for m in moments if m["m00"] > 0]
        # # Overlay centroids on the original image
        # path_image = original.copy()
        # for center in centroids:
        #     cv2.circle(path_image, center, 10, (0, 255, 0), -1)

        # # Step 6: Expect Turns (Turn Prediction)
        # height, width = gray.shape
        # front_black = black_mask[0:height // 3, :]
        # left_black = black_mask[:, 0:width // 3]
        # right_black = black_mask[:, 2 * width // 3 :]

        # front_detected = np.sum(front_black) > 1000
        # left_detected = np.sum(left_black) > 1000
        # right_detected = np.sum(right_black) > 1000

        # # if front_detected and not left_detected and not right_detected:
        # #     turn = "Dead End"
        # # elif front_detected and not right_detected:
        # #     turn = "Left Turn Expected"
        # # elif front_detected and not left_detected:
        # #     turn = "Right Turn Expected"
        # # else:
        # #     turn = "On Track"

        # # print(f"Turn Prediction: {turn}")

        # # Convert grayscale images to RGB (3 channels) for consistent display
        # white_mask_rgb = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2RGB)
        # black_mask_rgb = cv2.cvtColor(black_mask, cv2.COLOR_GRAY2RGB)
        # edges_rgb = cv2.cvtColor(edges, cv2.COLOR_GRAY2RGB)
        # masked_edges_rgb = cv2.cvtColor(masked_edges, cv2.COLOR_GRAY2RGB)
        # filtered_track_rgb = cv2.cvtColor(filtered_track, cv2.COLOR_GRAY2RGB)
        # line_image_rgb = cv2.cvtColor(line_image, cv2.COLOR_GRAY2RGB)
        # # Display all images in OpenCV window
        # combined_image = np.hstack([original, filtered_track_rgb, green_mask])
        # display_live_window(combined_image)
        cv2.imshow("preview", comp_img)
        # Check if 'q' is pressed to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Clean up
    cv2.destroyAllWindows()
    picam2.stop()