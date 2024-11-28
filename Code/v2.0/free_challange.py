import cv2
import numpy as np
from gpiozero import LED, Motor, Servo, Button
import time
from adaftimruit_servokit import ServoKit
# from calibration_v2 import load_hsv_from_json
import json
import os
# Initialize motor and servo for movement control
motor = Motor(forward=17, backward=27)
# servo = Servo(25)
kit = ServoKit(channels=8)

# Initialize the button on GPIO pin 18 (you can change this to your pin)
button = Button(23)

# Function to execute when the button is pressed
def on_button_press():
    print("Button was pressed!")

# Function to execute when the button is released
def on_button_release():
    global start_robot
    print("Button was released!")
    if start_robot:
        motor_stop()
    else:
        reset_all()
    start_robot = not start_robot
    time.sleep(1)
# Assign the function to the button press and release
button.when_pressed = on_button_press
button.when_released = on_button_release

# File to store the HSV values
json_file_path = 'hsv_values.json'

# Movement variables
orignal_speed = 0.6
steer_speed = 0.5
speed = orignal_speed
left_angle = 60
right_angle = 140
center_angle = 100
#robot direction
cw = 1
ccw = -1
robot_dir = 0
# parameters
start_robot = False
steer_lock_end_time = 0
# steering_ratio = 0.5
similarity_start_time = 0

# Threshold for frame similarity
similarity_threshold = 0.9  # Similarity percentage
similarity_duration = 3  # 3 seconds

# Store previous frame to compare
previous_frame = None
similar_frame_count_start = 0
time_before_shutdown = 0

# Initialize orange strip count
orange_strip_count = 0
orange_in_previous_frame = False  # Flag to track orange detection between frames
detect_orange_time = 0
blue_strip_count = 0
blue_in_previous_frame = False
detect_blue_time = 0
isBackward = False
# Function to move the car forward, backward, and stop
def motor_forward():
    global start_robot
    print("Move Forward")
    if start_robot:
        motor.forward(speed)

def motor_backward():
    print("Move Backward")
    if start_robot:
        motor.backward(speed)

def motor_stop():
    print("Stop")
    motor.stop()

# Servo control for steering
def steer_left():
    print("Steer Left")
    global speed
    speed = steer_speed
    # servo.value = -1 * steering_ratio
    if start_robot:
        kit.servo[0].angle = int(left_angle)


def steer_right():
    print("Steer Right")
    global speed
    speed = steer_speed
    # servo.value = steering_ratio
    if start_robot:
        kit.servo[0].angle = int(right_angle)


def steer_center():
    global speed
    speed = orignal_speed
    print("Steer Center")
    # servo.value = 0
    kit.servo[0].angle = int(center_angle)

backward_steering = steer_right
def counter_shutdown(current_time,delay):
    global start_robot
    if time.time()-time_before_shutdown > delay:
        start_robot = False
        motor_stop()
        steer_center()
        print("ooooooh yeaah")
        # exit()

# Function to load HSV values from the JSON file (if it exists)

# Color thresholds (HSV) loaded from JSON or defaults
color_ranges = {
    'black': {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 50]), 'label': 'Black'},
    'red': {'lower': np.array([0, 120, 70]), 'upper': np.array([10, 255, 255]), 'label': 'Red'},
    'green': {'lower': np.array([40, 70, 70]), 'upper': np.array([80, 255, 255]), 'label': 'Green'},
    'orange': {'lower': np.array([5, 100, 100]), 'upper': np.array([15, 255, 255]), 'label': 'Orange'},
    'blue': {'lower': np.array([100, 150, 0]), 'upper': np.array([140, 255, 255]), 'label': 'Blue'},
    'magenta': {'lower': np.array([100, 150, 0]), 'upper': np.array([140, 255, 255]), 'label': 'Magenta'}
}

def load_hsv_from_json():
    if os.path.exists(json_file_path):
        with open(json_file_path, 'r') as json_file:
            saved_data = json.load(json_file)
        
        # Load saved data into the color_ranges
        for mask in saved_data:
            color_ranges[mask]['lower'] = np.array(saved_data[mask]['lower'])
            color_ranges[mask]['upper'] = np.array(saved_data[mask]['upper'])
        # update_trackbar_values()

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
    width_middle_center = 150
    y_up = 25
    middle_center = ((center_x - width_middle_center, center_y - 50), (center_x + width_middle_center, center_y + 50))
    left_center = ((0, center_y - 25 - y_up), (center_x - 150, center_y + 50 - y_up))
    right_center = ((center_x + 150, center_y - 25 - y_up), (w, center_y + 50 - y_up))
    lower_middle = ((center_x - 50, center_y + 100), (center_x + 50, h))

    return middle_center, left_center, right_center, lower_middle

threshold_pixels_num = 300
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
    global robot_dir, time_before_shutdown, start_robot, orange_blue_counter, threshold_pixels_num, steer_lock_end_time, similarity_start_time, previous_frame, orange_strip_count, orange_in_previous_frame, blue_strip_count, blue_in_previous_frame, backward_steering
    global detect_blue_time, detect_orange_time
    # Detect all colors
    black_contours = detect_color_contours(hsv_frame, 'black')
    red_contours = detect_color_contours(hsv_frame, 'red')
    green_contours = detect_color_contours(hsv_frame, 'green')
    orange_contours = detect_color_contours(hsv_frame, 'orange')
    blue_contours = detect_color_contours(hsv_frame, 'blue')
    
    # Define rectangles
    middle_center, left_center, right_center, lower_middle = define_rectangles(frame.shape)

    # Draw rectangles on the frame (for visualization)
    cv2.rectangle(frame, middle_center[0], middle_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, left_center[0], left_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, right_center[0], right_center[1], (255, 255, 255), 2)
    cv2.rectangle(frame, lower_middle[0], lower_middle[1], (255, 255, 255), 2)

    # Draw and label contours
    draw_labeled_contours(frame, black_contours, color_ranges['black']['label'], (0, 0, 0))
    draw_labeled_contours(frame, red_contours, color_ranges['red']['label'], (0, 0, 255))
    draw_labeled_contours(frame, green_contours, color_ranges['green']['label'], (0, 255, 0))
    draw_labeled_contours(frame, orange_contours, color_ranges['orange']['label'], (0, 165, 255))
    draw_labeled_contours(frame, blue_contours, color_ranges['blue']['label'], (255, 0, 0))

    # Handle locked steering (car is locked in a steering direction)
    current_time = time.time()
    if current_time < steer_lock_end_time or not start_robot:
        # If steering is locked, do nothing else
        time_before_shutdown = current_time
        return frame    
    # Check for frame similarity over time (car is stuck)
    if frames_are_similar(previous_frame, frame):
        if similarity_start_time == 0:
            similarity_start_time = current_time
        elif current_time - similarity_start_time >= similarity_duration:
            # If frames are similar for 3 seconds, move the car backward
            motor_backward()
            time_before_shutdown = current_time
            backward_steering()
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
    
    # Prioritize closest contours
    if red_in_center and green_in_center:
        # If both red and green detected, steer based on the closest one
        if red_contours[0][0][0][1] < green_contours[0][0][0][1]:
            steer_right()
        else:
            steer_left()
    elif red_in_center:
        steer_right()
    elif green_in_center:
        steer_left()
    
    # Centering logic based on black contours
    # black_in_left = any(is_contour_in_rectangle(c, left_center) for c in black_contours)
    # black_in_right = any(is_contour_in_rectangle(c, right_center) for c in black_contours)
    black_in_left = are_pixels_in_rectangle(black_pixels, left_center)
    black_in_right = are_pixels_in_rectangle(black_pixels, right_center)
    black_in_lower = are_pixels_in_rectangle(black_pixels, lower_middle)

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
    else:
        steer_center()
    
    # Lower middle logic (for blue-orange pattern)
    orange_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in orange_contours)
    blue_in_lower = any(is_contour_in_rectangle(c, lower_middle) for c in blue_contours)
    # If the orange strip is detected for the first time (i.e., it wasn't detected in the previous frame)
    if orange_in_lower and not orange_in_previous_frame and current_time - detect_orange_time > 3:
        orange_strip_count =  blue_strip_count if robot_dir == ccw and blue_strip_count <= orange_strip_count else orange_strip_count+ 1
        detect_orange_time = current_time
        print(f"Orange strip passed: {orange_strip_count}")
        if blue_strip_count==0 and orange_strip_count== 1:
            robot_dir = cw
            backward_steering = steer_left

        if robot_dir == cw:
            steer_right()
            steer_lock_end_time = current_time + 0.1
    
    # If the orange strip is detected for the first time (i.e., it wasn't detected in the previous frame)
    if blue_in_lower and not blue_in_previous_frame and current_time - detect_blue_time > 3:
        blue_strip_count = orange_strip_count if robot_dir == cw and blue_strip_count >= orange_strip_count else blue_strip_count+ 1
        detect_blue_time = current_time
        print(f"blue strip passed: {blue_strip_count}")
        if blue_strip_count==1 and orange_strip_count== 0:
            robot_dir = ccw
            backward_steering = steer_right

        if robot_dir == ccw:
            steer_left()
            steer_lock_end_time = current_time + 0.1
    
    labs = 4*3
    if orange_strip_count >= labs and robot_dir == ccw:
        print("shutdowwwwwwn")
        counter_shutdown(current_time,3)
    elif blue_strip_count >= labs and robot_dir == cw:
        print("shutdowwwwwwn")
        counter_shutdown(current_time,3)
    else:
        time_before_shutdown = current_time

    # Update the flag to track the presence of the orange strip in the current frame
    blue_in_previous_frame = blue_in_lower
    orange_in_previous_frame = orange_in_lower

    # Backward adjustment based on other contours in lower rectangle
    # if any(is_contour_in_rectangle(c, lower_middle) for c in black_contours + red_contours + green_contours):
    # if any(is_contour_in_rectangle(c, lower_middle) for c in black_contours):
    if black_in_lower and black_in_left > threshold_pixels_num and black_in_right > threshold_pixels_num:
        # steer_left()
        backward_steering()
        motor_backward()
        time_before_shutdown = current_time
        steer_lock_end_time = time.time()
        steer_lock_end_time += 1
    else:
        motor_forward()

    return frame

def reset_all():
    global orange_strip_count, blue_strip_count, robot_dir, detect_orange_time, detect_blue_time, start_robot
    global steer_lock_end_time, similarity_start_time, previous_frame, similarity_start_time, orange_in_previous_frame
    global blue_in_previous_frame, backward_steering, time_before_shutdown

    # Reset movement variables
    orange_strip_count = 0
    blue_strip_count = 0
    robot_dir = 0  # Reset direction
    detect_orange_time = 0
    detect_blue_time = 0
    time_before_shutdown = 0
    start_robot = False  # Stop the robot
    steer_lock_end_time = 0
    similarity_start_time = 0
    previous_frame = None
    orange_in_previous_frame = False
    blue_in_previous_frame = False
    backward_steering = steer_right  # Reset to default backward steering

    # Reset hardware components
    motor_stop()        # Stop the motor
    steer_center()      # Center the servo
    
    print("System reset to default.")
# Main camera loop
cap = cv2.VideoCapture(0)

# Load HSV calibration data from the JSON file
load_hsv_from_json()
while True:
    ret, frame = cap.read()
    if not ret:
        break


    # Rotate and resize the frame for performance
    # frame = cv2.rotate(frame, cv2.ROTATE_180)
    frame = cv2.resize(frame, (400, 300))
    
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Drive the car based on contours and steering logic
    frame = drive_based_on_contours(frame, hsv_frame)

    # Display frame with drawn contours and rectangles
    # try:
    cv2.imshow("Autonomous Car", frame)
    # except :
    #     pass
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC to exit
        break
    elif key == ord('p'):
        if start_robot:
            motor_stop()
        else:
            reset_all()
        start_robot = not start_robot

cap.release()
cv2.destroyAllWindows()
