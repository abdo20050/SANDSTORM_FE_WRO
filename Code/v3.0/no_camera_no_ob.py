import cv2
import numpy as np
import time
from picamera2 import Picamera2
import json
import os
import traceback
import datetime
import sys

from motor_controller import create_dc_motor
from servo_controller import create_servo


def _init_camera(self):
        picam2 = Picamera2()
        video_config = picam2.create_video_configuration(main={"size": (500, 400), "format": "RGB888"})
        picam2.configure(video_config)
        picam2.start()
        return picam2
# from calibration_v2 import load_hsv_from_json


motor = create_dc_motor()  # Uses default pins 17 and 27
servo = create_servo(channel=7, max_pulse=2500, min_pulse=500, reverse_angle=False, max_angle=40)

# File to store the HSV values
json_file_path = 'hsv_values.json'

# Movement variables
original_speed = 0.17
steer_speed = 0.1
speed = original_speed
left_angle   = 60
right_angle  = -60
center_angle = 0

start_robot = False
steer_lock_end_time = 0
# steering_ratio = 0.5
similarity_start_time = 0

# Threshold for frame similarity
similarity_threshold = 0.9  # Similarity percentage
similarity_duration  = 3  # 3 seconds

last_backward_time = 0
backward_delay = 4

# Store previous frame to compare
previous_frame = None
similar_frame_count_start = 0

# Initialize orange strip count
orange_strip_count = 0
orange_in_previous_frame = False  # Flag to track orange detection between frames
blue_strip_count = 0
blue_in_previous_frame = False


lap_counter = 0

# Function to move the car forward, backward, and stop
def motor_forward(speed = speed):
    global start_robot
    print("Move Forward")
    if start_robot:
        motor.set_speed(speed)

def motor_backward():
    print("Move Backward")
    if start_robot:
        motor.set_speed(-speed)
        
def motor_backward_hit_wall():
    print("Move Backward hit wall")
    if start_robot:
        motor.set_speed(-speed)
        time.sleep(2)
        
    

def motor_stop():
    print("Stop")
    motor.stop()

# Servo control for steering
def steer_left(angle = left_angle):
    print("Steer Left")
    # servo.value = -1 * steering_ratio
    global speed
    speed = steer_speed
    servo.set_angle(left_angle)


def steer_right(angle = right_angle):
    print("Steer Right")
    # servo.value = steering_ratio
    global speed
    speed = steer_speed
    servo.set_angle(right_angle)



def steer_center():
    print("Steer Center")
    # servo.value = 0
    global speed
    speed = original_speed
    servo.center()





# Color thresholds (HSV) loaded from JSON or defaults
color_ranges = {
    'black': {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 50]), 'label': 'Black'},
    'red': {'lower': np.array([0, 120, 70]), 'upper': np.array([10, 255, 255]), 'label': 'Red'},
    'green': {'lower': np.array([40, 70, 70]), 'upper': np.array([80, 255, 255]), 'label': 'Green'},
    'orange': {'lower': np.array([5, 100, 100]), 'upper': np.array([15, 255, 255]), 'label': 'Orange'},
    'blue': {'lower': np.array([100, 150, 0]), 'upper': np.array([140, 255, 255]), 'label': 'Blue'},
    'maginta': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255]), 'label':'Magenta'}
}

# Pattern detection counters
orange_blue_counter = 0

# Minimum contour area threshold to filter small contours
MIN_CONTOUR_AREA = 700  # You can adjust this value as needed

# Function to create the mask and return the pixel positions within the masked region
def detect_color_pixels(hsv_frame, color):
    mask = cv2.inRange(hsv_frame, color_ranges[color]['lower'], color_ranges[color]['upper'])
    blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)
    # Apply thresholding after the blur
    _, thresh_mask = cv2.threshold(blurred_mask, 127, 255, cv2.THRESH_BINARY)
    # Extract the coordinates of non-zero pixels in the mask
    pixel_positions = np.argwhere(thresh_mask > 0)
    return pixel_positions, thresh_mask

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

# Function to define rectangles
def define_rectangles(frame_shape):
    h, w = frame_shape[:2]
    center_x = w // 2
    center_y = h // 2
    
    # Define key regions
    width_middle_center = 200
    middle_center = ((center_x - width_middle_center, center_y - 100), (center_x + width_middle_center, center_y + 100))
    left_center = ((0, center_y - 50), (center_x - 200, center_y + 50))
    right_center = ((center_x + 200, center_y - 50), (w, center_y + 50))
    left_block_detector = ((0, 250), (center_x - 200, h))
    right_block_detector = ((center_x + 200, 250), (w, h))
    lower_middle = ((center_x - 100, center_y + 150), (center_x + 100, h))

    return middle_center, left_center, right_center, left_block_detector, right_block_detector, lower_middle

threshold_pixels_num = 100
# Check if a set of pixels are in a specific rectangle
def are_pixels_in_rectangle(pixels, rectangle):
    (rx1, ry1), (rx2, ry2) = rectangle
    
    # Count the number of pixels that fall inside the rectangle
    pixels_in_rectangle = [p for p in pixels if rx1 <= p[1] <= rx2 and ry1 <= p[0] <= ry2]
    
    # Define a threshold for pixel count to consider it detected
    return len(pixels_in_rectangle)  # Adjust threshold based on testing

# Check if part of a contour is in a specific rectangle
def is_contour_in_rectangle(contour, rectangle):
    x, y, w, h = cv2.boundingRect(contour)
    (rx1, ry1), (rx2, ry2) = rectangle
    
    # Check if the bounding rectangle of the contour overlaps with the detection rectangle
    if x < rx2 and x + w > rx1 and y < ry2 and y + h > ry1:
        return True
    return False

# Function to draw bounding boxes and label contours
def draw_labeled_contours(frame, contours, color_label, color):
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
        cv2.putText(frame, color_label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
# Function to compare frames for similarity
def frames_are_similar(frame1, frame2, threshold=similarity_threshold):
    if frame1 is None or frame2 is None:
        return False

    # Convert to grayscale for simpler comparison
    gray1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Compute structural similarity
    difference = cv2.absdiff(gray1, gray2)
    _, diff_thresh = cv2.threshold(difference, 25, 255, cv2.THRESH_BINARY)

    # Calculate the ratio of non-difference pixels
    non_zero_count = np.count_nonzero(diff_thresh)
    total_pixels = frame1.shape[0] * frame1.shape[1]
    similarity_ratio = (total_pixels - non_zero_count) / total_pixels

    return similarity_ratio >= threshold

# Function to drive car based on contours
def drive_based_on_contours(frame, hsv_frame):
    global start_robot, orange_blue_counter, threshold_pixels_num, steer_lock_end_time, similarity_start_time, previous_frame, orange_strip_count, orange_in_previous_frame, blue_strip_count, blue_in_previous_frame
    global speed, last_backward_time, lap_counter,line_color_flag,main_detected_color
    
    # Detect all colors
    black_contours = detect_color_contours(hsv_frame, 'black')
    red_contours = detect_color_contours(hsv_frame, 'red')
    green_contours = detect_color_contours(hsv_frame, 'green')
    orange_contours = detect_color_contours(hsv_frame, 'orange')
    blue_contours = detect_color_contours(hsv_frame, 'blue')
    
    # Define rectangles
    middle_center, left_center, right_center, left_block_detector, right_block_detector, lower_middle = define_rectangles(frame.shape)

    # Draw rectangles on the frame (for visualization)
    cv2.rectangle(frame, middle_center[0], middle_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, left_center[0], left_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, right_center[0], right_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, left_block_detector[0], left_block_detector[1], (0, 0, 255), 1)
    cv2.rectangle(frame, right_block_detector[0], right_block_detector[1], (0, 255, 0), 1)
    cv2.rectangle(frame, lower_middle[0], lower_middle[1], (255, 255, 255), 2)

    # Draw and label contours
    draw_labeled_contours(frame, black_contours, color_ranges['black']['label'], (0, 0, 0))
    draw_labeled_contours(frame, red_contours, color_ranges['red']['label'], (0, 0, 255))
    draw_labeled_contours(frame, green_contours, color_ranges['green']['label'], (0, 255, 0))
    draw_labeled_contours(frame, orange_contours, color_ranges['orange']['label'], (0, 165, 255))
    draw_labeled_contours(frame, blue_contours, color_ranges['blue']['label'], (255, 0, 0))

    # Handle locked steering (car is locked in a steering direction)
    if start_robot == False:
        return frame
    current_time = time.time()
    if current_time < steer_lock_end_time :
        # If steering is locked, do nothing else
        return frame    
    # Check for frame similarity over time (car is stuck)
    if frames_are_similar(previous_frame, frame):
        if similarity_start_time == 0:
            similarity_start_time = current_time
        elif current_time - similarity_start_time >= similarity_duration :
            # If frames are similar for 3 seconds, move the car backward
            print("Stuckck")
            motor_backward()
            if (blue_strip_count < orange_strip_count):
                steer_right()
            else:
                steer_left()
            steer_lock_end_time = time.time()
            steer_lock_end_time += 1
            # time.sleep(2)  # Move backward for 2 seconds
            # motor_stop()
            similarity_start_time = 0  # Reset the timer
            return frame
    else:
        # Reset similarity timer if the frames are not similar
        similarity_start_time = 0

    # Store current frame as the previous frame for the next iteration
    previous_frame = frame.copy()
    
    # Detect all colors and get pixel positions
    black_pixels, black_mask = detect_color_pixels(hsv_frame, 'black')
    red_pixels, red_mask = detect_color_pixels(hsv_frame, 'red')
    green_pixels, green_mask = detect_color_pixels(hsv_frame, 'green')
    orange_pixels, orange_mask = detect_color_pixels(hsv_frame, 'orange')
    blue_pixels, blue_mask = detect_color_pixels(hsv_frame, 'blue')

    # Check contours in key regions
    red_in_center = any(is_contour_in_rectangle(c, middle_center) for c in red_contours)
    green_in_center = any(is_contour_in_rectangle(c, middle_center) for c in green_contours)
    red_in_right = any(is_contour_in_rectangle(c, right_center) for c in red_contours)
    green_in_left = any(is_contour_in_rectangle(c, left_center) for c in green_contours)
    green_in_right_desired = any(is_contour_in_rectangle(c, right_block_detector) for c in green_contours)
    red_in_left_desired = any(is_contour_in_rectangle(c, left_block_detector) for c in red_contours)
    red_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in red_contours)
    green_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in green_contours)
    # block_detected = red_in_center or green_in_center
    block_detected = red_in_center or green_in_center or green_in_left or red_in_right or green_in_lower or red_in_lower
    # Prioritize closest contours
    if red_in_center and green_in_center:
        # If both red and green detected, steer based on the closest one
        motor_forward()
        if red_contours[0][0][0][1] < green_contours[0][0][0][1]:
            steer_right()
        else:
            steer_left()
    elif red_in_center:
        motor_forward()
        steer_right()
        steer_lock_end_time = current_time + 0.5
    elif green_in_center:
        motor_forward()
        steer_left()
        steer_lock_end_time = current_time + 0.5
    elif green_in_left or green_in_lower:
        print("hit green block")
        motor_backward()
        steer_right()
        steer_lock_end_time = current_time + 1
        # return frame
    elif red_in_right or red_in_lower:
        print("hit green block")
        motor_backward()
        steer_left()
        steer_lock_end_time = current_time + 1
        # return frame
    elif green_in_right_desired:
        motor_forward()
        steer_left()
        steer_lock_end_time = current_time + 0.2
    elif  red_in_left_desired:
        motor_forward()
        steer_left()
        steer_lock_end_time = current_time + 0.2
    
    if block_detected:
        
        speed = steer_speed
        return frame
    else:
        speed = original_speed
    # Centering logic based on black contours
    # black_in_left = any(is_contour_in_rectangle(c, left_center) for c in black_contours)
    # black_in_right = any(is_contour_in_rectangle(c, right_center) for c in black_contours)
    black_in_left = are_pixels_in_rectangle(black_pixels, left_center)
    black_in_right = are_pixels_in_rectangle(black_pixels, right_center)
    black_in_low = are_pixels_in_rectangle(black_pixels, lower_middle)
    if(start_robot == True):
        if black_in_left> threshold_pixels_num and black_in_right > threshold_pixels_num:
            if black_in_left < black_in_right:
                steer_left()
            if black_in_left > black_in_right:
                steer_right()
            else:
                steer_center()
        elif black_in_left > threshold_pixels_num:
            steer_right()
        elif black_in_right > threshold_pixels_num:
            steer_left()
        elif not block_detected:
            steer_center()

    # Lower middle logic (for blue-orange pattern)
    orange_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in orange_contours)
    blue_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in blue_contours)
    # If the orange strip is detected for the first time (i.e., it wasn't detected in the previous frame)
    
    
    
    if blue_in_lower and not blue_in_previous_frame:
        blue_strip_count += 1
        if(line_color_flag == True):
            main_detected_color  = "BLUE"
            line_color_flag      = False
        motor.buzzer_on()
        time.sleep(0.05)
        motor.buzzer_off()
        print(f"Main detected Color: {main_detected_color}")
        if(blue_strip_count % 4 == 0):
            lap_counter +=1
        if blue_strip_count >= 13:
            motor_backward()
            time.sleep(1)
            print(f"blue strip passed: {blue_strip_count}")
            print(f"start_robot: {start_robot} -------------- it should stop")
            print(f"Lap: {lap_counter}")
            motor_stop()
            start_robot = False
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            time.sleep(0.05)
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            time.sleep(0.05)
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            sys.exit()
            

        else:
            print(f"blue strip passed: {blue_strip_count}")
            print(f"Lap: {lap_counter}")
            motor_backward()
            steer_left(10)
        
        steer_lock_end_time = current_time + 1

    elif orange_in_lower and not orange_in_previous_frame:
        orange_strip_count += 1
        if(line_color_flag == True):
            main_detected_color  = "ORANGE"
            line_color_flag      = False
        motor.buzzer_on()
        time.sleep(0.05)
        motor.buzzer_off()
        print(f"Main detected Color: {main_detected_color}")
        if(orange_strip_count % 4 == 0):
            lap_counter +=1
        if orange_strip_count >= 13:
            motor_backward()
            time.sleep(1)
            print(f"Orange strip passed: {orange_strip_count}")
            print(f"start_robot: {start_robot} -------------- it should stop")
            print(f"Lap: {lap_counter}")
            motor_stop()
            start_robot = False
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            time.sleep(0.05)
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            time.sleep(0.05)
            motor.buzzer_on()
            time.sleep(0.05)
            motor.buzzer_off()
            sys.exit()


        else:
            print(f"Orange strip passed: {orange_strip_count}")            
            print(f"Lap: {lap_counter}")
            motor_backward()
            steer_right(10)
        
        steer_lock_end_time = current_time + 1
        
        
        
        # orange_in_previous_frame = orange_in_lower
        # return frame
    # elif not orange_in_lower and orange_in_previous_frame and not block_detected and current_time - last_backward_time > backward_delay:
    #     last_backward_time = current_time
    #     steer_left()
    #     print("on uturn")
    #     motor_backward()
    #     steer_lock_end_time = current_time + 1.4
    #     orange_in_previous_frame = orange_in_lower
    #     return frame    
    # # Update the flag to track the presence of the orange strip in the current frame
    orange_in_previous_frame = orange_in_lower
    blue_in_previous_frame = blue_in_lower

    # Count the pattern of orange and blue appearing together
    if not (orange_in_lower or blue_in_lower) and (orange_in_lower or blue_in_lower):
        orange_blue_counter += 1

    # Backward adjustment based on other contours in lower rectangle
    # if any(is_contour_in_rectangle(c, lower_middle) for c in black_contours + red_contours + green_contours):
    # if any(is_contour_in_rectangle(c, lower_middle) for c in black_contours):
    
    if black_in_low > threshold_pixels_num+200:
        if (orange_strip_count < blue_strip_count):
            steer_right()
        else:
            steer_left()

        motor_backward_hit_wall()
        servo.center()
        time.sleep(0.5)
        
        steer_right()  #for anticlock change to right
        time.sleep(0.5)
        steer_left()
        
        print("hit wall")
        steer_lock_end_time = time.time()
        steer_lock_end_time += 1
    else:
        motor_forward()

    return frame
# Function to load HSV values from the JSON file (if it exists)
def load_hsv_from_json():
    if os.path.exists(json_file_path):
        with open(json_file_path, 'r') as json_file:
            saved_data = json.load(json_file)
        
        # Load saved data into the color_ranges
        for mask in saved_data:
            color_ranges[mask]['lower'] = np.array(saved_data[mask]['lower'])
            color_ranges[mask]['upper'] = np.array(saved_data[mask]['upper'])
        # update_trackbar_values()
        
# Main camera loop
cap = Picamera2()
video_config = cap.create_video_configuration(main={"size": (500, 400), "format": "RGB888"})
cap.configure(video_config)
cap.start()
load_hsv_from_json()


steer_center()
start_robot = True
line_color_flag = True

motor.buzzer_on()
time.sleep(0.07)
motor.buzzer_off()
time.sleep(0.07)
motor.buzzer_on()
time.sleep(0.07)
motor.buzzer_off()
time.sleep(0.07)
motor.buzzer_on()
time.sleep(0.07)
motor.buzzer_off()

def log_error(error_msg):
    timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    with open("car_error_log.txt", "a") as log_file:
        log_file.write(f"[{timestamp}] {error_msg}\n")


try:
    while True:
        try:
            frame = cap.capture_array()
            if frame is None:
                error_msg = "Failed to capture frame"
                print(error_msg)
                log_error(error_msg)
                break
            
            hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            # Drive the car based on contours and steering logic
            frame = drive_based_on_contours(frame, hsv_frame)
            
        except cv2.error as cv_err:
            error_msg = f"OpenCV Error: {cv_err}"
            print(error_msg)
            log_error(error_msg)
            continue
        
        except Exception as e:
            error_msg = f"An error occurred: {str(e)}\nError type: {type(e).__name__}\nTraceback:\n{traceback.format_exc()}"
            print(error_msg)
            log_error(error_msg)
            break

finally:
    # Ensure resources are released even if an error occurs
    cap.close()
    cv2.destroyAllWindows()

