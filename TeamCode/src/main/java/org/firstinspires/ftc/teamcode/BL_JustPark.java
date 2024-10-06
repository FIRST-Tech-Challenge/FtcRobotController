package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "Robot: Blue left just Park", group = "Robot")

public class BL_JustPark extends LinearOpMode {
    private BNO055IMU imu = null;      // Control/Expansion Hub IMU ==>this is for turnning

    private double robotHeading = 0;
    private double headingOffset = 0;
    private double headingError = 0;
    private double targetHeading = 0;
    static final double HEADING_THRESHOLD = 0.5;    // How close must the heading get to the target before moving to next step.

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeft = null;
    private DcMotor rearLeft = null;
    private DcMotor frontRight = null;
    private DcMotor rearRight = null;

    static final double COUNTS_PER_MOTOR_REV = 537.7;   // eg: GoBILDA 312 RPM Yellow Jacket
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.78;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //double maxTurnSpeed;
    // Adjust these numbers to suit your robot.

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    //private static final int DESIRED_TAG_ID = 5;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //*************************************************************
    //static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle was 0.01
    //    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 1.0;     // Maximum rotatiovf nal position was set to 1
    static final double MIN_POS = 0.0;     // Minimum rotational position

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double DRIVE_SPEED = 0.3;     // Max driving speed for better distance accuracy.

    static final double P_TURN_GAIN = 0.02;     // Larger is more responsive, but also less stable
    static final double P_DRIVE_GAIN = 0.03;     // Larger is more responsive, but also less stable
    private double turnSpeed = 0;

     public void runMotorsForTime(double y, double x, double rx, double time) {

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y - (x * 1.1) + rx) / denominator;
        double rearLeftPower = (y + (x * 1.1) + rx) / denominator;
        double frontRightPower = -(y - (x * 1.1) - rx) / denominator;
        double rearRightPower = -(y + (x * 1.1) - rx) / denominator;
        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < time)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.addData("move forward for 3 seconds", "Complete");
            telemetry.update();
        }
    }

    public void runMotorsWithTurning(double y, double x, double heading) {
        double rx = 0;
        double denominator = 0;
        double frontLeftPower = 0;
        double rearLeftPower = 0;
        double frontRightPower = 0;
        double rearRightPower = 0;
        turnSpeed=heading;

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);
        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
            // Determine required steering to keep on heading
            rx = getSteeringCorrection(heading, P_DRIVE_GAIN);
            // Clip the speed to the maximum permitted value.
            rx = Range.clip(rx, -0.2, 0.2);

            denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            frontLeftPower = (y + (x * 1.1) + rx) / denominator;
            rearLeftPower = (y - (x * 1.1) + rx) / denominator;
            frontRightPower = (y - (x * 1.1) - rx) / denominator;
            rearRightPower = (y + (x * 1.1) - rx) / denominator;
            frontLeft.setPower(frontLeftPower);
            rearLeft.setPower(rearLeftPower);
            frontRight.setPower(frontRightPower);
            rearRight.setPower(rearRightPower);
        }
        //Stop all motion;
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }


    @Override
    public void runOpMode()  {
        // Reverse the right side motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");

        // Reverse left motors if you are using NeveRests
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

                //lift
                DcMotor liftRight = hardwareMap.dcMotor.get("liftRight");
                DcMotor liftLeft = hardwareMap.dcMotor.get("liftLeft");
                //set direction of the lift motors be different directions
                liftRight.setDirection(DcMotorSimple.Direction.FORWARD);
                liftLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //IntakeWheel
        DcMotor IntakeWheel = hardwareMap.dcMotor.get("IntakeWheel");

        IntakeWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        //servoPlacer
        Servo servoPlacer;
        double MAX_POS = 1.0;
        double MIN_POS = 0.0;
        servoPlacer = hardwareMap.get(Servo.class, "servoPlacer");

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }
        //sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        //servo = hardwareMap.get(Servo.class, "GrabberServo");

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at fl rl fr rr", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                rearLeft.getCurrentPosition(),
                frontRight.getCurrentPosition(),
                rearRight.getCurrentPosition());
        telemetry.update();

        //        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
            telemetry.update();
        }
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        //***************take out later******************
        //encoderLeft(0.5,0,0,0,0,5);
        //***************************************************
        //put two pixel on the kickwheel and drop at the backstage  total score 11 point
        runMotorsForTime(0,0.3,0,3);//strafing to right
        encoderDrive(0.35,2,2,2,2,1);
        runMotorsWithTurning(0,0,-90);

        IntakeWheel.setPower(-0.5);
        sleep(2000);
        IntakeWheel.setPower(0);


        //tensorflow
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        //        //initVuforia();
        //        //initTfod();
        //        if (tfod != null) {
        //            tfod.activate();
        //
        //            // The TensorFlow software will scale the input images from the camera to a lower resolution.
        //            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
        //            // If your target is at distance greater than 50 cm (20") you can increase the magnification value
        //            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
        //            // should be set to the value of the images used to create the TensorFlow Object Detection model
        //            // (typically 16/9).
        //           tfod.setZoom(2.0);//magnification was at 1.0
        //        }
        //        if (opModeIsActive()) {
        //            while (opModeIsActive()) {
        //
        //
        //
        //
        //
        //                if (tfod != null) {
        ////                    telemetry.addData("begin of if statement", "1");
        //                // getUpdatedRecognitions() will return null if no new information is available since
        //                // the last time that call was made.
        //                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
        //                if (updatedRecognitions == null) {
        //                    continue;
        //                } else {
        //                    if (!updatedRecognitions.isEmpty()) {
        //                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
        //
        //                        // step through the list of recognitions and display image position/size information for each one
        //                        // Note: "Image number" refers to the randomized image orientation/number
        //                        //double col;
        //                        for (Recognition recognition : updatedRecognitions) {
        //                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
        //                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
        //                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
        //                            double height = Math.abs(recognition.getTop() - recognition.getBottom());
        //                            telemetry.addData("", " ");
        //                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
        //                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
        //                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
        //                            if (col > 160 & col < 480) {
        //
        //                                telemetry.addData("Detected!", "BlueAlianceCenter detected!");
        //                                telemetry.addLine("BlueAlianceCenterDetected!");
        //                                DESIRED_TAG_ID = 2;
        //                                //found the game piece in the center
        //                            }
        //                            if (col < 155) {
        //                                telemetry.addData("Detected!", "BlueAlianceLeft detected!");
        //                                telemetry.addLine("BlueAlianceLeft Detected!");
        //                                DESIRED_TAG_ID = 1;
        //                                //found the game piece in the Left
        //
        //                            }
        //                            if (col > 485) {
        //                                telemetry.addData("Detected!", "BlueAlianceRight detected!");
        //                                telemetry.addLine("BlueAlianceRight Detected!");
        //                                DESIRED_TAG_ID = 3;
        //                                //found the game piece in the Right
        //
        //                            }
        //                        }
        //
        ////                   **this part of code is working for tensorflow and Blue alliance center or ID2 for Apriltag
        //                        //if col is more than 490==> Blue alliance right or ID3
        //                        //if col is smaller than 150==>blue alliance left of Apriltag ID 1
        //                        telemetry.update();//
        // if (updatedRecognitions.get(0).getLabel() == "green") { //"Signal Sleeve 3")
        //runMotorsForTime(-0.4, 0, 0, 1.1);    //forward
        //runMotorsForTime(0,0,0,1);          //stop for 1 sec
        //runMotorsForTime(0,0.4,0,1.15); //move to left
        //WE START with Rear Camera facing the game element, So start with driving backward
        //then then right for the robot to drop the pixel on the front intake wheel on the
        //left spike mark with preloaded purple pixel
//        DESIRED_TAG_ID=5;
//        if (DESIRED_TAG_ID == 5) {
//            encoderDrive(0.25, 49, 49, 49, 49, 5.0);  // S1: backward 48 Inches with 5 Sec timeout
//
//            sleep(1000);
//            //drop pixel here
//            IntakeWheel.setPower(-0.3);
//            sleep(2000);
//            IntakeWheel.setPower(0);
//            encoderDrive(0.25, 5, 5, 5, 5, 2.0);
//
//
//            runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
//
//
//            //drive toward backdrop
//            encoderDrive(0.25, 22, 22, 22, 22, 5);
//            sleep(1000);
//            runMotorsForTime(0,-0.3,0,0.9);//strafing to right
//            sleep(1000);
//            encoderDrive(0.25, 12, 12, 12, 12, 5);
//
//            //runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
//            sleep(1000);
//            //drop the pixel on backdrop
//            double liftspeed = 0.25;
//            //lift goes up with runtime of 2 second
//            liftRight.setPower(liftspeed);
//            liftLeft.setPower(liftspeed);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 2)) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            //stop the lift
//            liftspeed = 0;
//            liftRight.setPower(liftspeed);
//            liftLeft.setPower(liftspeed);
//
//            //place the pixel
//            servoPlacer.setPosition(MIN_POS);
//            sleep(1000);
//            servoPlacer.setPosition(0.65*MAX_POS);
//            sleep(1000);
//            //lower the lift
//            liftspeed=0.25;
//            liftRight.setPower(-liftspeed);
//            liftLeft.setPower(-liftspeed);
//            runtime.reset();
//            while (opModeIsActive() && (runtime.seconds() < 2)) {
//                telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
//                telemetry.update();
//            }
//            encoderDrive(0.25, -2, -2, -2, -2, 2);
//
//            runMotorsForTime(0,-0.3,0,2);//park to the right of the field
//
//
//
//
//
//            //                targetFound = false;
////                desiredTag  = null;
//
//            // Step through the list of detected tags and look for a matching tag
////                List<AprilTagDetection> currentDetections = aprilTag.getDetections();
////                for (AprilTagDetection detection : currentDetections) {
////                    if ((detection.metadata != null) &&
////                            ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID))  ){
////                        targetFound = true;
////                        desiredTag = detection;
////                        break;  // don't look any further.
////                    } else {
////                        telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
////                    }
////                }
////
////                // Tell the driver what we see, and what to do.
////                if (targetFound) {
////                    telemetry.addData(">","HOLD Left-Bumper to Drive to Target\n");
////                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
////                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
////                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
////                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
////                } else {
////                    telemetry.addData(">","Drive using joysticks to find valid target\n");
////                }
////
////                // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
////                //if (gamepad1.left_bumper && targetFound) {
////                if (targetFound) {
//////we don't have gamepad so change it to test
////                    // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
////                    double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
////                    double  headingError    = desiredTag.ftcPose.bearing;
////                    double  yawError        = -desiredTag.ftcPose.yaw;//notice it is turning to the right given this is rear view camera
//////Y axis points straight outward from the camera lens center
//////
//////X axis points to the right, perpendicular to the Y axis
//////
//////Z axis points upward, perpendicular to Y and X
////                    // Use the speed and turn "gains" to calculate how we want the robot to move.
////                    drive  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
////                    turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
////                    strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
////
////                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
////                } else {
////
////                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//////                    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//////                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//////                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
////                    drive=0.25;
////                    strafe=0.25;
////                    turn=0.25;
////
////                    //telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
////                }
////                telemetry.update();
////
////                // Apply desired axes motions to the drivetrain.
////                moveRobot(drive, strafe*1.1, turn);//add 1.1 power
////                sleep(10);
//        }
    }
    /**
     * Initialize the AprilTag processor.
     */
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

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

        //backward drive to april tag

        // Then avoid the dropped pixel

        //runMotorsForTime(0, 0.4, 0, 0.50); //move to left
        // lift up, forward, drop pixel, backward, drop lift
        //                                runLiftEncoders(0.4, 16, 5);

        //                            runMotorsWithTurning(0,0,0);

//                                sleep(5000);
//                                encoderDrive(0.3, -6,-6,-6,-6,5);
//                                sleep(2000);
//                                //
        //    encoderDrive(DRIVE_SPEED, -3, -3, -3, -3, 5);
        //                                sleep(3000);
        //                                position = MAX_POS;
        //                                servo.setPosition(MAX_POS);
        //                                sleep(2000);
        //                                position =MIN_POS;
        //                                servo.setPosition(MIN_POS);
        //                                runLiftEncoders(0,0,2);

        //                                encoderDrive(DRIVE_SPEED,3,3,3,3,5);
        //                                runLiftEncoders(-0.4,-16,5);
        //                                //back to the position
        //                           runMotorsWithTurning(0, 0, 75); //rotate counterclockwise 75 degree to detect april tag
//                                encoderDrive(DRIVE_SPEED, 2, 2, 2, 2, 5.0);//back ward for 5 inch
//                                runMotorsWithTurning(0, 0, 90); //rotate clockwise 90 degree
//                              encoderDrive(DRIVE_SPEED, 20.5, 20.5, 20.5, 20.5, 5.0);
//    //                            telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));//back distance sensor
//                                //find april tab
//                                telemetry.update();
////                                break;

    }
//                    } else if (updatedRecognitions.get(0).getLabel() == "pink") {//"Sleeve 1")
//                        encoderDrive(DRIVE_SPEED, -29.5, -29.5, -29.5, -29.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//                        //   telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
//
////                                // Rev2mDistanceSensor specific methods.
////                                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
////                                //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
//                        telemetry.update();
//                        runMotorsWithTurning(0, 0, -45); //rotate clockwise 45 degree
//                        sleep(1000);
//                        runMotorsWithTurning(0, 0, 0); //rotate counter clockwise 0 degree
//                        encoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 5.0);
//                        //runMotorsWithTurning(0,0,-90); //rotate clockwise 90 degree
//                        //encoderDrive(DRIVE_SPEED,  -23.5,  -23.5, -23.5,-23.5,5.0);
//                        break;
//
//                    } else if (updatedRecognitions.get(0).getLabel() == "yellow") {//"Signal Sleeve 2") {
//                        //runMotorsForTime(-0.4, 0,0, 1.10);    //forward
//                        //runMotorsForTime(0, 0,0, 1);    //stop for 1 sec
//
//                        //runMotorsForTime(0, -0.4, 0, 1.15);    //to the right
//
//                        encoderDrive(DRIVE_SPEED, -29.5, -29.5, -29.5, -29.5, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
//                        // telemetry.addData("range", String.format("%.01f in", sensorRange.getDistance(DistanceUnit.INCH)));
//
////                                // Rev2mDistanceSensor specific methods.
////                                telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
////                                //telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
//                        telemetry.update();
//                        runMotorsWithTurning(0, 0, -45); //rotate clockwise 45 degree
//                        sleep(1000);
//                        runMotorsWithTurning(0, 0, 0); //rotate counter clockwise 0 degree
//                        encoderDrive(DRIVE_SPEED, -18, -18, -18, -18, 5.0);
//                        runMotorsWithTurning(0, 0, -90); //rotate clockwise 90 degree
//                        encoderDrive(DRIVE_SPEED, -19, -19, -19, -19, 5.0);
//                        break;
//                    }
//                }

/**
 //forward straight at -0.4 speed==>26.5 or 27 in/sec
 //strafing right -0.4 speed==> measure 16.5 inch/sec but maybe 19.5 to 20 inch/sec. When use 23.5/20.4=1.15
 //backward at 0.4 speed==> measure 26.5 in/sec
 //strafing left at 0.4 speed==> measuer??? inch . but maybe the speed is about 23.5/19.5=1.20
 //rotation at 0.2 speed==>//left rotation 0.75 rotation 0.75 rotation/1.5 sec=0.5 rotation/sec
 */
    //zone 1 code ==>go forward 29.5 inch=>strafing left 23.5 inch



//        }
    // step through the list of recognitions and display image position/size information for each one
    // Note: "Image number" refers to the randomized image orientation/number

//                    telemetry.update();
//                }
//
//                telemetry.addData("Path", "Complete");
//                telemetry.update();
//                sleep(1000);
//            }

    // ElapsedTime runtime = new ElapsedTime();


//    private void initVuforia() {
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         */
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        //  Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//    }

    /**
     * Initialize the TensorFlow Object Detection engine.


    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading   The desired absolute heading (relative to last heading reset)
     * @param proportionalGain Gain factor applied to heading error to obtain turning power.
     * @return Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double fleftInches, double frightInches,
                             double rleftInches, double rrightInches,
                             double timeoutS) {
        int newfLeftTarget;
        int newrLeftTarget;
        int newfRightTarget;
        int newrRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLeftTarget = frontLeft.getCurrentPosition() + (int) (fleftInches * COUNTS_PER_INCH);
            newfRightTarget = frontRight.getCurrentPosition() + (int) (frightInches * COUNTS_PER_INCH);
            newrLeftTarget = rearLeft.getCurrentPosition() + (int) (rleftInches * COUNTS_PER_INCH);
            newrRightTarget = rearRight.getCurrentPosition() + (int) (rrightInches * COUNTS_PER_INCH);


            frontLeft.setTargetPosition(newfLeftTarget);
            frontRight.setTargetPosition(newfRightTarget);
            rearLeft.setTargetPosition(newrLeftTarget);
            rearRight.setTargetPosition(newrRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newfLeftTarget, newfRightTarget, newrLeftTarget, newrRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(), rearRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderLeft(double speed,
                            double fleftInches, double frightInches,
                            double rleftInches, double rrightInches,
                            double timeoutS) {
        int newfLeftTarget;
        int newrLeftTarget;
        int newfRightTarget;
        int newrRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newfLeftTarget = frontLeft.getCurrentPosition(); //+ (int)(fleftInches * COUNTS_PER_INCH);
            newfRightTarget = frontRight.getCurrentPosition();// + (int)(frightInches * COUNTS_PER_INCH);
            newrLeftTarget = rearLeft.getCurrentPosition();// + (int)(rleftInches * COUNTS_PER_INCH);
            newrRightTarget = rearRight.getCurrentPosition();// + (int)(rrightInches * COUNTS_PER_INCH);

            //this following code moved strafing to the right 11 inches
            frontLeft.setTargetPosition(newfLeftTarget - 500);
            frontRight.setTargetPosition(newfRightTarget + 500);
            rearLeft.setTargetPosition(newrLeftTarget + 500);
            rearRight.setTargetPosition(newrRightTarget - 500);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            rearLeft.setPower(Math.abs(speed));
            rearRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (frontLeft.isBusy() && frontRight.isBusy() && rearLeft.isBusy() && rearRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newfLeftTarget, newfRightTarget, newrLeftTarget, newrRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), rearLeft.getCurrentPosition(), rearRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            rearLeft.setPower(0);
            rearRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    /**
     * This method will be used to run the motor encoders for right lift because the leftlift motor encoder is broken
     *
     * @param speed
     * @param Inches   \
     * @param timeOutS
     */
    public void runLiftEncoders(double speed, double Inches, long timeOutS) {
        // Declare variables to hold targets (the counts we want the motors to run to)
        int Target1 = 0;


    }
}

