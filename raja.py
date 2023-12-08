import cv2
import numpy as np
import math
import sys
import time
import time
import board
import adafruit_mcp4728
import matplotlib.pyplot as plt


MCP4728_DEFAULT_ADDRESS = 0x60
MCP4728A4_DEFAULT_ADDRESS = 0x64

i2c = board.I2C()  # uses board.SCL and board.SDA
mcp4728 = adafruit_mcp4728.MCP4728(i2c, adafruit_mcp4728.MCP4728A4_DEFAULT_ADDRESS)

# forward = 28500
# stop_drive = 25000

# voltage
init = 35225
forward = 34300 #driving
stop_drive = 30000

# wheel direction
straight = 30000
left = 15500 # 15000
right = 49500 

passedStopLight = False
atStopLight = False
passedFirstStopSign = False

def detect_edges(frame):
    # filter for blue lane lines
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype = "uint8")
    upper_blue = np.array([150, 255, 255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue)
    #cv2.imshow("mask",mask)
    
    # detect edges
    edges = cv2.Canny(mask, 50, 100)
    #cv2.imshow("edges",edges)
    
    return edges

def region_of_interest(edges):
    height, width = edges.shape
    mask = np.zeros_like(edges)

    # only focus lower half of the screen
    polygon = np.array([[
        # (0, height/4),
        # (0,  3*height/4),
        # (width , height/4),
        # (width , 3*height/4),
        (0, height),
        (0, height / 2),
        (width, height / 2),
        (width, height),
    ]], np.int32)
    
    cv2.fillPoly(mask, polygon, 255)
    
    cropped_edges = cv2.bitwise_and(edges, mask)
    cv2.imshow("roi",cropped_edges)
    
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10  
    
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=150)

    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []
    
    if line_segments is None:
        # print("no line segments detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1/3
    left_region_boundary = width * (1 - boundary)
    right_region_boundary = width * boundary
    
    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                #print("skipping vertical lines (slope = infinity")
                continue
            
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)
            
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    
    slope, intercept = line
    
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down
    
    if slope == 0:
        slope = 0.1
        
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    
    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6):
    line_image = np.zeros_like(frame)
    
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)
                
    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)
    
    return line_image


def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5 ):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape
    
    steering_angle_radian = steering_angle / 180.0 * math.pi
    
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)
    
    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)
    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)
    
    return heading_image

def get_steering_angle(frame, lane_lines):
    
    height,width,_ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        # print("left_x2", left_x2)
        # print("right_x2", right_x2)
        # print("mid", mid)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
        
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
        
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle

def go_faster():
    mcp4728.channel_c.value = int(max(30000,mcp4728.channel_c.value + 100))
    #print("fast",mcp4728.channel_c.value)
def go_slower():
    mcp4728.channel_c.value = int(mcp4728.channel_c.value - 1000)
    #print("fast",mcp4728.channel_c.value)

def getRedFloorBoundaries():
    """
    Gets the hsv boundaries and success boundaries indicating if the floor is red
    :return: [[lower color and success boundaries for red floor], [upper color and success boundaries for red floor]]
    """
    return getBoundaries("final_project/vals.txt")


def detect_stopsign(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    
    #red color has two ranges in hsv
    lower_red1 = np.array([0, 70, 20])
    upper_red1 = np.array([15, 255, 255])
    
    lower_red2 = np.array([150, 70,20])
    upper_red2 = np.array([179,255,255])
    
    red_mask_lower = cv2.inRange(hsv, lower_red1, upper_red1)
    red_mask_upper = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = red_mask_lower + red_mask_upper
    red = cv2.bitwise_and(image, image, mask = red_mask)
    total_pixels = red.size
    #if there is a significant amount of red, recognize it as a stopsign/light
    red_pixels = np.count_nonzero(red)
    percent = (red_pixels / total_pixels) * 100
    if percent > 25:
        print("percentage of red color: ", percent)
        return True
    return False

def isRedFloorVisible(frame):
    """
    Detects whether or not the floor is red
    :param frame: Image
    :return: [(True is the camera sees a red on the floor, false otherwise), video output]
    """
    # print("Checking for floor stop")
    boundaries = getRedFloorBoundaries()
    return isMostlyColor(frame, boundaries)

def isMostlyColor(image, boundaries):
    """
    Detects whether or not the majority of a color on the screen is a particular color
    :param image:
    :param boundaries: [[color boundaries], [success boundaries]]
    :return: boolean if image satisfies provided boundaries, and an image used for debugging
    """
    #Convert to HSV color space
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #parse out the color boundaries and the success boundaries
    color_boundaries = boundaries[0]
    percentage = boundaries[1]

    lower = np.array(color_boundaries[0])
    upper = np.array(color_boundaries[1])
    mask = cv2.inRange(hsv_img, lower, upper)
    output = cv2.bitwise_and(hsv_img, hsv_img, mask=mask)

    #Calculate what percentage of image falls between color boundaries
    percentage_detected = np.count_nonzero(mask) * 100 / np.size(mask)
    print("percentage color deteched")
    # print("percentage_detected " + str(percentage_detected) + " lower " + str(lower) + " upper " + str(upper))
    # If the percentage percentage_detected is betweeen the success boundaries, we return true, otherwise false for result
    result = percentage[0] < percentage_detected <= percentage[1]
    if result:
        # print(percentage_detected)
        pass
    return result, output

def getBoundaries(filename):
    """
    Reads the boundaries from the file filename
    Format:
        [0] lower: [H, S, V, lower percentage for classification of success]
        [1] upper: [H, S, V, upper percentage for classification of success]
    :param filename: file containing boundary information as above
    :return: [[lower color and success boundaries], [upper color and success boundaries]]
    """
    default_lower_percent = 50
    default_upper_percent = 100
    with open(filename, "r") as f:
        boundaries = f.readlines()
        lower_data = [val for val in boundaries[0].split(",")]
        upper_data = [val for val in boundaries[1].split(",")]

        if len(lower_data) >= 4:
            lower_percent = float(lower_data[3])
        else:
            lower_percent = default_lower_percent

        if len(upper_data) >= 4:
            upper_percent = float(upper_data[3])
        else:
            upper_percent = default_upper_percent

        lower = [int(x) for x in lower_data[:3]]
        upper = [int(x) for x in upper_data[:3]]
        boundaries = [lower, upper]
        percentages = [lower_percent, upper_percent]
    return boundaries, percentages

def plot_pd(p_vals, d_vals, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")

    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")

    if show_img:
        plt.show()
    plt.clf()


def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed PWM")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering PWM")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")

    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PWM Values")
    ax2.set_ylabel("Error Value")

    plt.title("PWM Values over time")
    fig.legend()
    plt.savefig("pwm_plot.png")

    if show_img:
        plt.show()
    plt.clf()

video = cv2.VideoCapture(0)
video.set(cv2.CAP_PROP_FRAME_WIDTH,320)
video.set(cv2.CAP_PROP_FRAME_HEIGHT,240)

time.sleep(1)

# fourcc = cv2.VideoWriter_fourcc(*'XVID')
# out = cv2.VideoWriter('Original15.avi',fourcc,10,(280,200))
# out2 = cv2.VideoWriter('Direction15.avi',fourcc,10,(280,200))

speed = 8
lastTime = 0
lastError = 0

kp = 0.4
kd = kp * 0.65

stopSignCheck = 1
sightDebug = False
isStopSignBool = False

max_ticks = 2000
counter = 0

# mcp4728.channel_c.value = stop_drive
mcp4728.channel_b.value = straight
mcp4728.channel_c.value = init

error_vals = []
p_vals = []
d_vals = []
speed_vals = []
steer_vals = []

while True:
    ret,original_frame = video.read()
    #frame = cv2.flip(frame,-1)
    frame = cv2.resize(original_frame, (160, 120))
    cv2.imshow("original",frame)
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    cv2.imshow("heading line",heading_image)

    now = time.time()

    time_diff = 5
    
    with open("/sys/module/gpiod_driver/parameters/elapsed_ms", "r") as filetoread:
        time_diff = int(filetoread.read()) / 100000
    if time_diff >= 30:
        mcp4728.channel_c.value = forward + 1000
        print("fast",mcp4728.channel_c.value)
    elif time_diff < 30 and time_diff > 5:
        mcp4728.channel_c.value = forward - 1000
        print("slow",mcp4728.channel_c.value)

    if ((counter + 1) % stopSignCheck) == 0:
        # check for the first stop sign
        if not passedFirstStopSign:
            # print("hasn't detected first stop sign")
            # isStopSignBool, floorSight = isRedFloorVisible(frame)
            
            isStopSignBool = detect_stopsign(frame)
            # print("current stop sign bool: ", isStopSignBool)
            # if sightDebug:
            #     cv2.imshow("floorSight", floorSight)
            if isStopSignBool:
                print("detected first stop sign, stopping")
                #stop()
                mcp4728.channel_c.value = stop_drive
                time.sleep(2)
                passedFirstStopSign = True
                # this is used to not check for the second stop sign until many frames later
                secondStopSignTick = counter + 200
                # now check for stop sign less frequently
                stopSignCheck = 3
                # add a delay to calling go faster
                #go_faster_tick = counter + go_faster_tick_delay
                print("first stop finished!")
                mcp4728.channel_c.value = forward
                mcp4728.channel_c.value = forward
                mcp4728.channel_c.value = forward
                # go_faster()
                # go_faster()
        # check for the second stop sign
        # elif passedFirstStopSign and isRedFloorVisible(frame)[0]:
        #         print("second stop sign detected, stopping")
        #         mcp4728.channel_c.value = stop_drive
        #         time.sleep(5)
        #         break
        elif passedFirstStopSign and counter > secondStopSignTick:
            isStop2SignBool = detect_stopsign(frame)
            if isStop2SignBool:
                # last stop sign detected, exits while loop
                print("second stop sign detected, stopping")
                mcp4728.channel_c.value = stop_drive
                time.sleep(5)
                break

    dt = now - lastTime

    deviation = steering_angle - 90
    error = abs(deviation)
    
    if deviation < 5 and deviation > -5:
        deviation = 0
        error = 0
        mcp4728.channel_b.value = straight

    elif deviation > 5:
        mcp4728.channel_b.value = right

    elif deviation < -5:
        mcp4728.channel_b.value = left 

    derivative = kd * (error - lastError) / dt
    proportional = kp * error
    PD = int(speed + derivative + proportional)
    spd = abs(PD)

    if spd > 25:
        spd = 25

    error_vals.append(error)
    p_vals.append(proportional)
    d_vals.append(derivative)
    speed_vals.append(mcp4728.channel_c.value )
    steer_vals.append(mcp4728.channel_b.value )

    lastError = error
    lastTime = time.time()
        
    # out.write(frame)
    # out2.write(heading_image)
    
    key = cv2.waitKey(1)
    if key == 27:
        break

    counter += 1



video.release()
cv2.destroyAllWindows()

mcp4728.channel_c.value = stop_drive
mcp4728.channel_b.value = straight

plot_pd(p_vals, d_vals, error_vals, True)
plot_pwm(speed_vals, steer_vals, error_vals, True)