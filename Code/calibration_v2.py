import cv2
import numpy as np
import os
import json
from gpiozero import LED, Motor, Servo
from pynput import keyboard
from adafruit_servokit import ServoKit

# A set to store currently pressed keys
pressed_keys = set()

def on_press(key):
    try:
        pressed_keys.add(key.char)
    except AttributeError:
        pass

def on_release(key):
    try:
        pressed_keys.remove(key.char)
    except KeyError:
        pass
    
    if key == keyboard.Key.esc:
        # Stop listener if ESC is pressed
        return False

# Start the keyboard listener in the background
listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

motor = Motor(forward=17, backward=27) 
kit = ServoKit(channels=8)

# servo = Servo(25)  # Pin for controlling the steering servo motor
speed = 0.5
def motor_forward():
    motor.forward(speed)  # 70% speed forward

def motor_backward():
    motor.backward(speed)  # 70% speed backward

def motor_stop():
    motor.stop()

# Servo motor control for steering
steering_ratio = 1
left_angle = 50
right_angle = 150
center_angle = 100
def steer_left():
    # Set servo to a position for left turn (-1 is full left)
    # servo.value = -1 * steering_ratio
    kit.servo[0].angle = int(left_angle)
def steer_right():
    # Set servo to a position for right turn (1 is full right)
    # servo.value = steering_ratio
    kit.servo[0].angle = int(right_angle)
def steer_center():
    # Set servo back to center (0 is center position)
    # servo.value = 0
    kit.servo[0].angle = int(center_angle)


# HSV ranges for the colors
color_ranges = {
    'black': {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 50])},
    'orange': {'lower': np.array([5, 100, 100]), 'upper': np.array([15, 255, 255])},
    'blue': {'lower': np.array([100, 150, 0]), 'upper': np.array([140, 255, 255])},
    'green': {'lower': np.array([40, 100, 100]), 'upper': np.array([70, 255, 255])},
    'red': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
    'magenta': {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])}
}

selected_mask = 'black'  # Default mask

# File to store the HSV values
json_file_path = 'hsv_values.json'

# Function to adjust the HSV bounds by 10% based on the clicked pixel value
def adjust_hsv_range(hsv_value, color_name):
    global color_ranges

    # Adjust by ±10% for both lower and upper bounds
    adjustment = hsv_value * 0.1

    new_lower = hsv_value - adjustment
    new_upper = hsv_value + adjustment

    # Ensure the values are within the valid HSV bounds (0-180 for H, 0-255 for S and V)
    color_ranges[color_name]['lower'] = np.clip(new_lower, [0, 0, 0], [180, 255, 255]).astype(int)
    color_ranges[color_name]['upper'] = np.clip(new_upper, [0, 0, 0], [180, 255, 255]).astype(int)

    # Update trackbars after adjusting the HSV values
    update_trackbar_values()
# Function to save the HSV values to a JSON file
def save_hsv_to_json():
    data_to_save = {mask: {'lower': color_ranges[mask]['lower'].tolist(), 'upper': color_ranges[mask]['upper'].tolist()} for mask in color_ranges}
    
    if os.path.exists(json_file_path):
        # If the file exists, load it, update it, and save back
        with open(json_file_path, 'r') as json_file:
            saved_data = json.load(json_file)
        
        # Update only the selected mask's data
        saved_data = data_to_save
        
        # Save updated data back to the file
        with open(json_file_path, 'w') as json_file:
            json.dump(saved_data, json_file, indent=4)
    else:
        # Save new data if the file doesn't exist
        with open(json_file_path, 'w') as json_file:
            json.dump(data_to_save, json_file, indent=4)
    
    print(f"Saved HSV values to {json_file_path}")

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
# Function to update the trackbars with the new HSV values
def update_trackbar_values():
    cv2.setTrackbarPos("Lower H", "Calibration", color_ranges[selected_mask]['lower'][0])
    cv2.setTrackbarPos("Lower S", "Calibration", color_ranges[selected_mask]['lower'][1])
    cv2.setTrackbarPos("Lower V", "Calibration", color_ranges[selected_mask]['lower'][2])
    cv2.setTrackbarPos("Upper H", "Calibration", color_ranges[selected_mask]['upper'][0])
    cv2.setTrackbarPos("Upper S", "Calibration", color_ranges[selected_mask]['upper'][1])
    cv2.setTrackbarPos("Upper V", "Calibration", color_ranges[selected_mask]['upper'][2])

# Mouse callback function
def pick_color(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = param  # HSV frame passed as parameter
        hsv_value = hsv_frame[y, x]  # Get the HSV value of the clicked pixel

        # Adjust the HSV range of the selected mask based on the clicked pixel
        adjust_hsv_range(hsv_value, selected_mask)

# Function to create trackbars and update the values
def create_trackbar_window():
    cv2.namedWindow("Calibration")

    cv2.createTrackbar("Lower H", "Calibration", color_ranges[selected_mask]['lower'][0], 180, lambda v: None)
    cv2.createTrackbar("Lower S", "Calibration", color_ranges[selected_mask]['lower'][1], 255, lambda v: None)
    cv2.createTrackbar("Lower V", "Calibration", color_ranges[selected_mask]['lower'][2], 255, lambda v: None)
    cv2.createTrackbar("Upper H", "Calibration", color_ranges[selected_mask]['upper'][0], 180, lambda v: None)
    cv2.createTrackbar("Upper S", "Calibration", color_ranges[selected_mask]['upper'][1], 255, lambda v: None)
    cv2.createTrackbar("Upper V", "Calibration", color_ranges[selected_mask]['upper'][2], 255, lambda v: None)

# Function to update HSV values from the trackbars
def update_hsv_from_trackbar():
    color_ranges[selected_mask]['lower'][0] = cv2.getTrackbarPos("Lower H", "Calibration")
    color_ranges[selected_mask]['lower'][1] = cv2.getTrackbarPos("Lower S", "Calibration")
    color_ranges[selected_mask]['lower'][2] = cv2.getTrackbarPos("Lower V", "Calibration")
    color_ranges[selected_mask]['upper'][0] = cv2.getTrackbarPos("Upper H", "Calibration")
    color_ranges[selected_mask]['upper'][1] = cv2.getTrackbarPos("Upper S", "Calibration")
    color_ranges[selected_mask]['upper'][2] = cv2.getTrackbarPos("Upper V", "Calibration")

# Function to show both the original frame and the color mask side-by-side
def show_dual_frame(original_frame, mask):
    combined_frame = np.hstack((original_frame, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)))
    cv2.imshow('Calibration', combined_frame)

# Function to detect and draw contours on the original frame
def draw_contours_on_original(original_frame, mask):
    # Find contours
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours on the original frame
    contour_frame = original_frame.copy()
    cv2.drawContours(contour_frame, contours, -1, (0, 255, 0), 2)  # Green contours with thickness of 2
    
    return contour_frame

# Function to overlay text on the frame
def overlay_text(frame, text):
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    color = (255, 255, 255)  # White text
    thickness = 2
    cv2.putText(frame, f"Adjusting: {text}", (10, 30), font, font_scale, color, thickness, cv2.LINE_AA)

if __name__ == '__main__':
    # Initialize camera
    cap = cv2.VideoCapture(0)

    # Load HSV values from JSON if the file exists
    load_hsv_from_json()

    # Create a window for the current selected mask calibration
    create_trackbar_window()

    while True:
        # Capture frame from the camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame")
            break

        # Rotate the frame 180 degrees
        # frame = cv2.rotate(frame, cv2.ROTATE_180)

        # Resize for performance
        frame = cv2.resize(frame, (400, 300))
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Update HSV values from the trackbars
        update_hsv_from_trackbar()

        # Create the mask based on the current HSV range
        mask = cv2.inRange(hsv_frame, color_ranges[selected_mask]['lower'], color_ranges[selected_mask]['upper'])

        # Apply Gaussian Blur to the mask
        blurred_mask = cv2.GaussianBlur(mask, (5, 5), 0)

        # Apply thresholding after the blur
        _, thresh_mask = cv2.threshold(blurred_mask, 127, 255, cv2.THRESH_BINARY)

        # Draw contours on the original frame
        contour_frame = draw_contours_on_original(frame, thresh_mask)

        # Overlay text indicating which color mask is being adjusted
        overlay_text(contour_frame, selected_mask.capitalize())

        # Show the dual frame (left: original frame with contours, right: thresholded mask)
        show_dual_frame(contour_frame, thresh_mask)

        # Set mouse callback to the original frame
        cv2.setMouseCallback('Calibration', pick_color, hsv_frame)

        # Keyboard input for switching between color masks
        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            selected_mask = 'black'
            create_trackbar_window()
        elif key == ord('2'):
            selected_mask = 'orange'
            create_trackbar_window()
        elif key == ord('3'):
            selected_mask = 'blue'
            create_trackbar_window()
        elif key == ord('4'):
            selected_mask = 'green'
            create_trackbar_window()
        elif key == ord('5'):
            selected_mask = 'red'
            create_trackbar_window()
        elif key == ord('6'):
            selected_mask = 'magenta'
            create_trackbar_window()
        elif key == ord('j'):
            save_hsv_to_json()  # Save the HSV values to the JSON file
        elif key == ord('q'):  # Exit calibration mode
            break

        #steering
        if 'd' in pressed_keys:
            print("steer right")
            steer_right()
        elif 'a' in pressed_keys:
            print("steer left")
            steer_left()
        else:
            steer_center()
        #forward and backward
        if 'w' in pressed_keys:
            motor_forward()
        elif 's' in pressed_keys:
            motor_backward()
        else:
            motor_stop()

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    exit()
    listener.stop()