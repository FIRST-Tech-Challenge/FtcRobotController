
    package archive;

    import com.qualcomm.robotcore.eventloop.opmode.Disabled;
    import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.hardware.IMU;
    import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.util.Range;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
    import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
    import org.firstinspires.ftc.vision.VisionPortal;
    import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
    import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
    import com.qualcomm.robotcore.hardware.AccelerationSensor;

    import java.util.List;
    import java.util.concurrent.TimeUnit;



    @TeleOp(name="MAIN Omni Drive AprilTag")

    @Disabled
    public class Main extends LinearOpMode{
        //Kõik staatilised väärtused ja nende tüübid nt int, float, imu
        
        final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.3;   //  Clip the turn speed to this max value (adjust for your robot)

        private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
        private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
        private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
        private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

        private Servo gimbalPitch = null;
        private Servo gimbalYaw = null;

        private static final boolean USE_WEBCAM = false;  // Set true to use a webcam, or false for a phone camera
        private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
        private VisionPortal visionPortal;               // Used to manage the video source.
        private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
        private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        IMU imu;
        AccelerationSensor accelerationSensor;
        @Override public void runOpMode(){
            //kõik muutujad ja inisisaliseerimine

            double currentPitch = 0.5;
            double currentYaw = 0.5;
            boolean targetFound     = false;    // Set to true when an AprilTag target is detected
            double  drive           = 0;        // Desired forward power/speed (-1 to +1)
            double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
            double  turn            = 0;        // Desired turning power/speed (-1 to +1)

            // Initialize IMU
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(
                new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
                )
            );

            // Initialize the Apriltag Detection process
            initAprilTag();

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must match the names assigned during the robot configuration.
            // step (using the FTC Robot Controller app on the phone).
            leftFrontDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_0_CH");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "Motor_Port_2_CH");
            leftBackDrive  = hardwareMap.get(DcMotor.class, "Motor_Port_1_CH");
            rightBackDrive = hardwareMap.get(DcMotor.class, "Motor_Port_3_CH");

            gimbalPitch = hardwareMap.get(Servo.class, "Servo_Port_0_CH");
            gimbalPitch.setPosition(currentPitch);
            gimbalYaw = hardwareMap.get(Servo.class, "Servo_Port_1_CH");
            gimbalYaw.setPosition(currentYaw);

            // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
            // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
            // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
            leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
            rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

            if (USE_WEBCAM)
                setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
                
            imu.resetYaw();
            // Wait for driver to press start
            telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
            telemetry.addData(">", "Touch Play to start OpMode");
            telemetry.update();
            waitForStart();

            while (opModeIsActive())
            {
                targetFound = false;
                desiredTag  = null;

                // Step through the list of detected tags and look for a matching tag
                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (AprilTagDetection detection : currentDetections) {
                    // Look to see if we have size info on this tag.
                    if (detection.metadata != null) {
                        //  Check to see if we want to track towards this tag.
                        if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                            // Yes, we want to use this tag. 
                            targetFound = true;
                            desiredTag = detection;
                            break;  // don't look any further.
                        } else {
                            // This tag is in the library, but we do not want to track it right now.
                            telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                        }
                    } else {
                        // This tag is NOT in the library, so we don't have enough information to track to it.
                        telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                    }
                }

                // Tell the driver what we see, and what to do.
                if (targetFound) {
                    telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                    telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData("\n>","Drive using joysticks to find valid target\n");
                }

                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
                if (gamepad1.left_bumper && targetFound) {

                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                    double  headingError    = desiredTag.ftcPose.bearing;
                    double  yawError        = desiredTag.ftcPose.yaw;

                    // Use the speed and turn "gains" to calculate how we want the robot to move.
                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                if (gamepad1.dpad_up) { 
                    currentPitch += 0.01;
                    gimbalPitch.setPosition(currentPitch);
                }
                if (gamepad1.dpad_down) {
                    currentPitch -= 0.01;
                    gimbalPitch.setPosition(currentPitch);
                }
                if (gamepad1.dpad_left) {
                    currentYaw -= 0.01;
                    gimbalYaw.setPosition(currentYaw);
                }
                if (gamepad1.dpad_right) {
                    currentYaw += 0.01;
                    gimbalYaw.setPosition(currentYaw);
                }

                //tractionControl();
                // Apply desired axes motions to the drivetrain.

                moveRobot(drive, strafe, turn);
                sleep(10);
            }
        }
        /**
         * set desired power to motors
         * this method(function) will remove excess power to prevent slip
         
        public void tractionControl(){
                Acceleration acceleration = accelerationSensor.getAcceleration();
                telemetry.addData("X acceleration: ", acceleration.xAccel);
                telemetry.addData("Y acceleration: ", acceleration.yAccel);
                telemetry.addData("Z acceleration: ", acceleration.zAccel);
        }*/


        /**
         * Move robot according to desired axes motions
         * <p>
         * Positive X is forward
         * <p>
         * Positive Y is strafe left
         * <p>
         * Positive Yaw is counter-clockwise
         */
        public void moveRobot(double x, double y, double yaw) {
            double max = Math.max(Math.abs(x) + Math.abs(y), 1);
            if (!gamepad1.left_bumper) {
                double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + Math.PI / 2;
                double tempx = x * Math.cos(heading) - y * Math.sin(heading);
                double tempy = x * Math.sin(heading) + y * Math.cos(heading);
                x = tempx;
                y = tempy;
            }

            // Calculate wheel powers.
            double leftFrontPower    =  (-x + y -yaw)/max/1.16;
            double leftBackPower     =  (x + y -yaw)/max/1.16;
            double rightFrontPower   =  (x + y +yaw)/max;
            double rightBackPower    =  (-x + y +yaw)/max;

            
            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightFrontDrive.setPower(rightFrontPower);
            rightBackDrive.setPower(rightBackPower);
        }

            /**
         * Initialize the AprilTag processor.
         */
        private void initAprilTag() {
            // Create the AprilTag processor by using a builder.
            aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // eg: Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            aprilTag.setDecimation(2);

            // Create the vision portal by using a builder.
            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }

        /*
        Manually set the camera gain and exposure.
        This can only be called AFTER calling initAprilTag(), and only works for Webcams;
        */
        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }

            // Make sure camera is streaming before we try to set the exposure controls
            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
    }
