import cv2  # OpenCV Library
import numpy as np  # Numpy Library
import serial  # PySerial Library
import math

# Initialize capture feed to webcam and set output resolution to 1280 x 720
cap = cv2.VideoCapture(2)
cap.set(3, 1280)
cap.set(4, 720)

# Initialize pyserial communication on COM9 at baud rate of 115200. Clear output buffer on entry.
bluetooth = serial.Serial('COM9', 115200)
bluetooth.reset_output_buffer()

# Initialize robot and lego coordinate variables that store string for serial communication
GlobalCenterRobotX = 0
GlobalCenterRobotY = 0
GlobalArmRobotX = 0
GlobalArmRobotY = 0
GlobalLegoX = 0
GlobalLegoY = 0

# Initialize coordinates variables for holding integer values of converted blob coordinates
CenterRobotX = 0
CenterRobotY = 0
ArmRobotX = 0
ArmRobotY = 0
LegoX = 0
LegoY = 0

# Initialize four separate simpleblobdetector filter parameters.
# Center robot blob, arm robot blob, small lego blobs and large lego blobs
CenterRobotParams = cv2.SimpleBlobDetector_Params()  # Green blob
ArmRobotParams = cv2.SimpleBlobDetector_Params()  # Blue blob
SmallLegoParams = cv2.SimpleBlobDetector_Params()  # Red small blob
LargeLegoParams = cv2.SimpleBlobDetector_Params()  # Reg large blob

# Green Center Robot Detect Parameters
CenterRobotParams.filterByColor = True
CenterRobotParams.blobColor = 0
CenterRobotParams.filterByArea = True
CenterRobotParams.minArea = 100
CenterRobotParams.maxArea = 2000

# Robot Blue Circle Detect Parameters
ArmRobotParams.filterByColor = True
ArmRobotParams.blobColor = 0
ArmRobotParams.filterByArea = True
ArmRobotParams.minArea = 150
ArmRobotParams.maxArea = 1000

# Small Lego Detect Parameters
SmallLegoParams.filterByColor = True
SmallLegoParams.blobColor = 0
SmallLegoParams.filterByArea = True
SmallLegoParams.minArea = 160
SmallLegoParams.maxArea = 200

# Large Lego Detect Parameters
LargeLegoParams.filterByColor = True
LargeLegoParams.blobColor = 0
LargeLegoParams.filterByArea = True
LargeLegoParams.minArea = 350
LargeLegoParams.maxArea = 1000

# Initialize SimpleBlobDetector for Robot and Legos with respective parameters
CenterRobotDetector = cv2.SimpleBlobDetector_create(CenterRobotParams)
ArmRobotDetector = cv2.SimpleBlobDetector_create(ArmRobotParams)
SmallLegoDetector = cv2.SimpleBlobDetector_create(SmallLegoParams)
LargeLegoDetector = cv2.SimpleBlobDetector_create(LargeLegoParams)

# While the webcam is available the following loop will be true
while (cap.isOpened()):

    # Set webcam feed to the variable frame. Ret will return true if this line was possible.
    # Frame var for robot blob detection
    ret, frame = cap.read()

    # Field var for lego blob detection. Will have rectangles over drop areas.
    ret2, field = cap.read()

    if ret == True:
        # Initialize the font to be used when writing text to webcam feed window.
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Drawing two black rectangles in areas of drop zones. This is so already dropped legos are not detected again.
        # Coordinates are in pixels as per arguments 2 and 3 of 'cv2.rectangle' function.
        field = cv2.rectangle(field, (137,583), (332,720), (0, 0, 0), -1)
        field = cv2.rectangle(field, (935, 595), (1142, 720), (0, 0, 0), -1)

        # Detection of Lego Blocks using HSV filter method
        hsv = cv2.cvtColor(field, cv2.COLOR_BGR2HSV)  # Converting the field webcam feed window to HSV. Store in hsv var
        l_b = np.array([0, 130, 0])  # The lower bounds of hsv mask. Args are as followed: hue, saturation, value
        u_b = np.array([12, 255, 255])  # The upper bounds of hsv mask. Args are as followed: hue, saturation, value
        mask = cv2.inRange(hsv, l_b, u_b)  # Create a mask of the hsv window with the upper and lower hsv bounds
        # res = cv2.bitwise_and(field, field, mask=mask) # Bitwise and operation of the field window with hsv mask
        reversemask = 255-mask  # The mask var will have a black background and white blobs. This computes the opposite
        blur = cv2.GaussianBlur(reversemask, (9, 9), 0)  # The reverse mask is noisy. Gaussian blur for smoother output

        # Detection of Blue Robot Blob Arm using HSV filter method
        hsv1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_b1 = np.array([88, 30, 77])
        u_b1 = np.array([130, 255, 255])
        mask1 = cv2.inRange(hsv1, l_b1, u_b1)
        # res = cv2.bitwise_and(field, field, mask=mask)
        reversemask1 = 255-mask1
        blur1 = cv2.GaussianBlur(reversemask1, (9, 9), 0)

        # Detection of Center Green Robot Blob using HSV filter method
        hsv2 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_b2 = np.array([35, 0, 83])
        u_b2 = np.array([83, 255, 255])
        mask2 = cv2.inRange(hsv1, l_b2, u_b2)
        # res = cv2.bitwise_and(field, field, mask=mask)
        reversemask2 = 255-mask2
        blur2 = cv2.GaussianBlur(reversemask2, (9, 9), 0)

        # Detector.detect(field) will find the blobs in the given window based off the filters and input params.
        CenterRobotKeyPoints = CenterRobotDetector.detect(blur2)
        ArmRobotKeyPoints = ArmRobotDetector.detect(blur1)
        SmallLegoKeyPoints = SmallLegoDetector.detect(blur)
        LargeLegoKeyPoints = LargeLegoDetector.detect(blur)

        # Draw circles around detected blobs. Robot blob will have red circle. Lego blobs will have green circle.
        # Should maybe change the drawn keypoints for robot
        frame = cv2.drawKeypoints(frame, CenterRobotKeyPoints, np.array([]), (0, 255, 0),
                                  cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
        frame = cv2.drawKeypoints(frame, ArmRobotKeyPoints, np.array([]), (255, 0, 0),
                                  cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
        frame = cv2.drawKeypoints(frame, SmallLegoKeyPoints, np.array([]), (0, 111, 255),
                                  cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)
        frame = cv2.drawKeypoints(frame, LargeLegoKeyPoints, np.array([]), (0, 247, 255),
                                  cv2.DrawMatchesFlags_DRAW_RICH_KEYPOINTS)

        # If there is a small Lego detected in the FOV, this statement will hold true.
        if len(SmallLegoKeyPoints) > 0:
            # Retrieve the X and Y Pixel Location of first small Lego in the keypoints array.
            LegoXPixel = SmallLegoKeyPoints[0].pt[0]
            LegoYPixel = SmallLegoKeyPoints[0].pt[1]

            # If there are two legos, complete comparator to choose Lego closest to starting blue zone
            # Comparing x coordinate relative to pixels. I.e. y value of testing area frame.
            if len(SmallLegoKeyPoints) == 2:
                if SmallLegoKeyPoints[1].pt[0] < LegoXPixel:
                    LegoXPixel = SmallLegoKeyPoints[1].pt[0]
                    LegoYPixel = SmallLegoKeyPoints[1].pt[1]

            # Function created in matlab converting pixel location to a scalar multiplier.
            LegoXPixelIndex = (0.0119*LegoXPixel) - 1.5632
            LegoYPixelIndex = (0.0119*LegoYPixel) + 0.4245

            # Function created for converting pixel to real X and Y values in millimeters.
            # Note that LegoXPixel is really the Y coordinate relative to robot field and vice versa.
            LegoY = -200 + (200*LegoXPixelIndex) - ((2.5**-14)*LegoYPixelIndex)
            LegoX = -200 + (200*LegoYPixelIndex) - ((1.3**-14)*LegoXPixelIndex)

            # Writing the Lego's coordinate on the field window for debugging.
            LegoLocation = 'LegoX: ' + str(LegoX) + '  LegoY: ' + str(LegoY)
            cv2.putText(frame, LegoLocation, (800, 15), font, 0.5, (101, 235, 215), 2)

            # Need to format coordinates found as a 6 digit number for repeatability, 2 of which are decimals.
            # If the coordinates found are less than a meter then the first number will be a 0.
            GlobalLegoX = "{:07.2f}".format(LegoX)
            GlobalLegoY = "{:07.2f}".format(LegoY)

        elif len(LargeLegoKeyPoints) > 0:

            # Retrieve the X and Y Pixel Location of first Large Lego in the keypoints array.
            LegoXPixel = LargeLegoKeyPoints[0].pt[0]
            LegoYPixel = LargeLegoKeyPoints[0].pt[1]

            # If there are two legos, complete comparator to choose Lego closest to starting blue zone
            # Comparing x coordinate relative to pixels. I.e. y value of testing area frame.
            if len(LargeLegoKeyPoints) == 2:
                if LargeLegoKeyPoints[1].pt[0] < LegoXPixel:
                    LegoXPixel = LargeLegoKeyPoints[1].pt[0]
                    LegoYPixel = LargeLegoKeyPoints[1].pt[1]

            # Function created in matlab converting pixel location to a scalar multiplier.
            LegoXPixelIndex = (0.0119*LegoXPixel) - 1.5632
            LegoYPixelIndex = (0.0119*LegoYPixel) + 0.4245

            # Function created for converting pixel to real X and Y values in millimeters.
            # Note that LegoXPixel is really the Y coordinate relative to robot field and vice versa.
            LegoY = -200 + (200*LegoXPixelIndex) - ((2.5**-14)*LegoYPixelIndex)
            LegoX = -200 + (200*LegoYPixelIndex) - ((1.3**-14)*LegoXPixelIndex)

            # Writing the first Lego's coordinate on the field window for debugging.
            LegoLocation = 'LegoX: ' + str(LegoX) + '  LegoY: ' + str(LegoY)
            cv2.putText(frame, LegoLocation, (800, 15), font, 0.5, (101, 235, 215), 2)

            # Need to format coordinates found as a 6 digit number for repeatability, 2 of which are decimals.
            # If the coordinates found are less than a meter then the first number will be a 0.
            GlobalLegoX = "{:07.2f}".format(LegoX)
            GlobalLegoY = "{:07.2f}".format(LegoY)

        else:
            # If no legos are found sending in dummy values.
            GlobalLegoX = GlobalLegoX
            GlobalLegoY = GlobalLegoY

        # If the Robots circle is detected in the FOV, this statement will hold true.
        if len(CenterRobotKeyPoints) > 0:
            # Retrieve the X and Y Pixel Location of Robot in the keypoints array.
            CRobotXPixel = CenterRobotKeyPoints[0].pt[0]
            CRobotYPixel = CenterRobotKeyPoints[0].pt[1]

            # Function created in matlab converting pixel location to a scalar multiplier.
            CRobotXPixelIndex = (0.0119*CRobotXPixel) - 1.5632
            CRobotYPixelIndex = (0.0119*CRobotYPixel) + 0.4245

            # Function created for converting pixel to real X and Y values in millimeters.
            # Note that RobotXPixel is really the Y coordinate relative to robot field and vice versa.
            CenterRobotY = -200 + (200*CRobotXPixelIndex) - ((2.5**-14)*CRobotYPixelIndex)
            CenterRobotX = -200 + (200*CRobotYPixelIndex) - ((1.3**-14)*CRobotXPixelIndex)

            # Writing the Robot's coordinate on the field window for debugging.
            RobotLocation = 'RobotX: ' + str(CenterRobotX) + ' RobotY: ' + str(CenterRobotY)
            cv2.putText(frame, RobotLocation, (800, 35), font, 0.5, (101, 235, 215), 2)

            GlobalCenterRobotX = CenterRobotX
            GlobalCenterRobotY = CenterRobotY

        else:
            GlobalCenterRobotX = CenterRobotX
            GlobalCenterRobotY = CenterRobotY

        if len(ArmRobotKeyPoints) > 0:
            # Retrieve the X and Y Pixel Location of Robot in the keypoints array.
            ARobotXPixel = ArmRobotKeyPoints[0].pt[0]
            ARobotYPixel = ArmRobotKeyPoints[0].pt[1]

            # Function created in matlab converting pixel location to a scalar multiplier.
            ARobotXPixelIndex = (0.0119*ARobotXPixel) - 1.5632
            ARobotYPixelIndex = (0.0119*ARobotYPixel) + 0.4245

            # Function created for converting pixel to real X and Y values in millimeters.
            # Note that RobotXPixel is really the Y coordinate relative to robot field and vice versa.
            ArmRobotY = -200 + (200*ARobotXPixelIndex) - ((2.5**-14)*ARobotYPixelIndex)
            ArmRobotX = -200 + (200*ARobotYPixelIndex) - ((1.3**-14)*ARobotXPixelIndex)

            GlobalArmRobotX = ArmRobotX
            GlobalArmRobotY = ArmRobotY

        else:
            # used to equal last global
            GlobalArmRobotX = ArmRobotX
            GlobalArmRobotY = ArmRobotY

        # cv2.imshow function will show the desired filtered or unfiltered webcam feed with a window name.

        cv2.imshow('Webcam Feed', frame)
        # cv2.imshow('field', field)
        # cv2.imshow('CenterBlob', blur2)
        # cv2.imshow('LegoBlock', blur)
        # cv2.imshow('ArmBlob', blur1)

        # Using the found coordinates of the robot with respect to driving area frame, compute heading of robot.
        CurrentAngle = math.atan2(GlobalArmRobotY - GlobalCenterRobotY, GlobalArmRobotX - GlobalCenterRobotX)
        CurrentAngle = math.degrees(CurrentAngle)

        GlobalCenterRobotX = "{:07.2f}".format(GlobalCenterRobotX)
        GlobalCenterRobotY = "{:07.2f}".format(GlobalCenterRobotY)
        CurrentAngle = "{:07.2f}".format(CurrentAngle)

        # Store lego coordinates, robot coordinates and robot heading into array to be sent
        GlobalLocation = [GlobalLegoX, GlobalLegoY, GlobalCenterRobotX, GlobalCenterRobotY,
                          CurrentAngle]

        GlobalLocation = str(GlobalLocation)  # Convert array to a string.
        GlobalLocation = GlobalLocation.replace("'", "")  # Str converter puts apostrophes around #, replace with null.
        print(GlobalLocation)  # Print converted array for debugging.
        bluetooth.write(str.encode(GlobalLocation))
        bluetooth.flush()

        if cv2.waitKey(1) & 0xFF == 27:  # If escape key is pressed, escape from if condition
            break

    else:  # Will break from while loop
        break

cap.release()  # Release all cameras
cv2.destroyAllWindows()  # Destroy all windows
