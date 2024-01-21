package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.rev.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.robotcore.external.tfod.*;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.apriltag.*;
import org.firstinspires.ftc.vision.tfod.*;
import java.util.*;

public abstract class CSBase extends LinearOpMode {
    public static final boolean USE_WEBCAM = true;
    public TfodProcessor tfod;
    public final ElapsedTime runtime = new ElapsedTime();
    // All non-primitve datatypes initialze to null on default.
    public DcMotorEx lf, lb, rf, rb, carWashMotor, pixelLiftingMotor;
    public Servo droneServo, pixelBackServo, pixelFrontServo, trayTiltingServo;
    public WebcamName camera;
    public TouchSensor touchSensor;
    public IMU imu;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //*/
    static final double     COUNTS_PER_MOTOR_REV    = ((((1+(46.0/17))) * (1+(46.0/11))) * 28) ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static       double     VELOCITY                = 2000;
    static final double     TILE_LENGTH             = 23.25;
    static final double     STRAFE_FRONT_MODIFIER   = 1.3;
    //static final double     VEL_MODIFIER            = 1.12485939258;
    static final double     b                       = 1.1375;
    static final double     m                       = 0.889;
    static final double     TURN_SPEED              = 0.5;
    static final boolean    TURN_TYPE               = false;
    public final double[]   boundaries              = {0, 350};
    double                  carWashPower            = 1.0;
    spike pos; // Team prop position
    public double x;
    public VisionPortal visionPortal;
    public static String TFOD_MODEL_ASSET;
    public AprilTagProcessor tagProcessor;

    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "prop",
    };
    IMU.Parameters imuparameters;

    /** Color options for the team prop. Options: red, blue, none **/
    public enum color {
        red, blue, none
    }

    /** Strafing directions. Options: left, right **/
    public enum dir {
        left,right
    }

    /** Spike mark positions for the team prop. Options: left, middle, right **/
    public enum spike {
        left,middle,right,none
    }

    /** Initializes all hardware devices on the robot.
     * @param teamColor The color of the team prop.
     * @param useCam Should the camera be initialized? **/
    public void setup(color teamColor, boolean useCam) {
        if (teamColor == color.red) {
            TFOD_MODEL_ASSET = "CSTeamPropRed.tflite";
        } else if (teamColor == color.blue){
            TFOD_MODEL_ASSET = "CSTeamPropBlue.tflite";
        }
        else if (useCam){
            telemetry.addData("", "Team color not specified, will not use team prop detection!");
            useCam = false;
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imuparameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                /*new Orientation(
                        AxesReference.INTRINSIC,
                        AxesOrder.ZYX,
                        AngleUnit.DEGREES,
                        0,
                        0,
                        0,
                        0
                )*/
        ));
        if (!imu.initialize(imuparameters)){
            telemetry.addData("IMU","Initialization failed");
        }
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront");
            lb = hardwareMap.get(DcMotorEx.class, "leftBack");
            rf = hardwareMap.get(DcMotorEx.class, "rightFront");
            rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        } catch (Exception e) {except(e); lf = lb = rf = rb = null;}
        // If given an error, the motor is already null
        try {carWashMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");}catch (Exception e){except(e);}
        try {pixelLiftingMotor = hardwareMap.get(DcMotorEx.class,"pixelLiftingMotor");}catch (Exception e){except(e);}
        try {droneServo = hardwareMap.get(Servo.class, "droneServo");}catch (Exception e){except(e);}
        try {pixelBackServo = hardwareMap.get(Servo.class,"pixelBackServo");}catch (Exception e){except(e);}
        try {pixelFrontServo = hardwareMap.get(Servo.class, "pixelFrontServo");}catch (Exception e){except(e);}
        try {trayTiltingServo = hardwareMap.get(Servo.class,"trayTiltingServo");}catch (Exception e){except(e);}
        try {touchSensor = hardwareMap.get(TouchSensor.class,"touchSensor");}catch (Exception e){except(e);}
        if (useCam) {
            try {
                camera = hardwareMap.get(WebcamName.class, "Webcam 1");
                initProcessors();
            } catch (Exception e) {except(e);}
        }

        if (lf != null) {
            lf.setDirection(DcMotorEx.Direction.REVERSE);
            lb.setDirection(DcMotorEx.Direction.REVERSE);
            rf.setDirection(DcMotorEx.Direction.FORWARD);
            rb.setDirection(DcMotorEx.Direction.FORWARD);

            lf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            lb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rf.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            rb.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            lb.setTargetPosition(lb.getCurrentPosition());
            rb.setTargetPosition(rb.getCurrentPosition());
            lf.setTargetPosition(lf.getCurrentPosition());
            rf.setTargetPosition(rf.getCurrentPosition());
        }

        if (pixelLiftingMotor != null) {
            pixelLiftingMotor.setDirection(DcMotorEx.Direction.REVERSE);
            pixelLiftingMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            pixelLiftingMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            pixelLiftingMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        }


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
    }

    /** Initializes all hardware devices on the robot.
     * Note: When called without useCam manually set, useCam defaults to true.
     * @param teamColor The color of the team prop. **/
    public void setup(color teamColor){
        setup(teamColor, true);
    }

    /** Initializes all hardware devices on the robot.
     * Note:
     * When called without useCam manually set, useCam defaults to true.
     * @param isRed Is the team prop red? **/
    public void setup(boolean isRed){
        if (isRed) {
            setup(color.red, true);
        }
        else {
            setup(color.blue, true);
        }
    }

    /** Drives using encoder velocity.
     * @param inches Amount of inches to drive. **/
    public void encoderDrive(double inches) {
        int lfTarget = 0;
        int rfTarget = 0;

        // Ensure that the OpMode is still active
        if (opModeIsActive() && lf != null) {
            lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION for front motors
            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();
            lb.setVelocity(VELOCITY * signum(inches));
            rb.setVelocity(VELOCITY * signum(inches));
            lf.setVelocity(VELOCITY * signum(inches));
            rf.setVelocity(VELOCITY * signum(inches));

            inches = signum(inches) * (abs(inches) + b) / m;

            double duration = abs(inches * COUNTS_PER_INCH / VELOCITY);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < duration)) {
                // Display it for the driver.
                telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Running to",  " %7d :%7d", lfTarget,  rfTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());
                telemetry.update();
            }

            stopRobot();

            // Turn off RUN_TO_POSITION
            // Note: Following code is technically redundant since called in stopRobot(), but the function
            // may be changed, so do not delete.
            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }

    /** Turns the robot a specified number of degrees. Positive values turn right,
     * negative values turn left.
     * @param degrees The amount of degrees to turn.
     */
    public void turn(double degrees) {
        sleep(100);
        imu.resetYaw();
        double tolerance = 1;
        if (TURN_TYPE) { // Boolean determines the method the robot takes to turn x degrees
            encoderDrive(degrees / 7.5);
            stopRobot();
        } else {
            degrees *= -1;
            double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double currentAngle;
            double initialGoalAngle = startAngle + degrees;
            double correctedGoalAngle = initialGoalAngle;
            double difference = 999;
            double turnModifier;
            double turnPower;
            if (abs(initialGoalAngle) > 180) {
                correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
            }
            while (opModeIsActive() && (difference > tolerance)) {
                currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                difference = min(abs(initialGoalAngle - currentAngle), abs(correctedGoalAngle - currentAngle));
                turnModifier = min(1, (difference + 3) / 30);
                turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier;
                lb.setPower(-turnPower);
                rb.setPower(turnPower);
                lf.setPower(-turnPower);
                rf.setPower(turnPower);
                telemetry.addData("Corrected Goal", correctedGoalAngle);
                telemetry.addData("Initial Goal", initialGoalAngle);
                telemetry.addData("Start", startAngle);
                telemetry.addData("Angle", currentAngle);
                telemetry.addData("Distance from goal", difference);
                telemetry.update();
            }
            stopRobot();
        }
    }

    /** Strafes left or right for a specified number of inches.
     * @param inches Amount of inches to strafe.
     * @param direction Direction to strafe in.**/
    public void strafe(double inches, dir direction /*double duration*/) {

        if (opModeIsActive() && lf != null) {
            lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

            double d;

            if (direction == dir.right) {
                d = -1;
            } else {
                d = 1;
            }


            runtime.reset();
            lb.setVelocity(VELOCITY * d);
            rb.setVelocity(-VELOCITY * d);
            lf.setVelocity(-VELOCITY * STRAFE_FRONT_MODIFIER * d);
            rf.setVelocity(VELOCITY * STRAFE_FRONT_MODIFIER * d);

            inches = (abs(inches) + 1.0125) / 0.7155;

            double duration = abs(inches * COUNTS_PER_INCH / VELOCITY);


            while (opModeIsActive() && (runtime.seconds() < duration)) {
                telemetry.addData("Strafing until",  duration + " seconds");
                telemetry.addData("Currently at",  runtime.seconds() + " seconds");
                telemetry.update();
            }

            stopRobot();

            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }
    }

    /** Strafes left or right until an april tag with the ID specified is detected.
     * @param direction Direction to strafe in.
     * @param idOfTag ID of April Tag to detect. **/
    public void strafeUntilTagDetection(dir direction, int idOfTag) {

        if (opModeIsActive() && lf != null) {
            lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            double d;

            if (direction == dir.right) {
                d = -1;
            } else {
                d = 1;
            }


            runtime.reset();
            lb.setVelocity(VELOCITY * d);
            rb.setVelocity(-VELOCITY * d);
            lf.setVelocity(-VELOCITY * STRAFE_FRONT_MODIFIER * d);
            rf.setVelocity(VELOCITY * STRAFE_FRONT_MODIFIER * d);

            while (opModeIsActive() && !detectTag(idOfTag)){
                telemetry.addData("Currently","Strafing until tag " + idOfTag + " is detected");
                telemetry.update();
            }

            stopRobot();

            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        }
    }

    /** Changes the velocity.
     * @param velocity New velocity value.
     */
    public void setSpeed(double velocity) {
        VELOCITY = velocity;
    }


    /** Drives the specified number of inches. Negative values will drive backwards.
     * @param inches Amount of inches to drive. **/
    public void drive(double inches) {
        //double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        for(int i = 0; i < checks; i++) {
            encoderDrive(inches / checks);
            //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }
        stopRobot();
    }

    /** Converts an amount of tiles on the game board to an amount of inches.
     * @param tiles The value of tiles to be converted. **/
    public double tiles(double tiles) {
        return tiles * TILE_LENGTH;
    }

    /** Makes the car wash outtake for 1 second. **/
    public void ejectPixel() {
        if (carWashMotor != null) {
            telemetry.addData("Car Wash", "Ejecting Pixel");
            telemetry.update();
            int t = (int) runtime.milliseconds() + 1000;
            carWashMotor.setPower(carWashPower);
            while (opModeIsActive()) {
                if (!(t > ((int) runtime.milliseconds()))) {
                    break;
                }
            }
            carWashMotor.setPower(0);
        } else {
            telemetry.addData("Car Wash", "Not Connected");
            telemetry.update();
        }
    }

    /** Stops all drive train motors on the robot. **/
    public void stopRobot() {
        // Set target position to avoid an error
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        lb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Stop all motion
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        lb.setPower(0);
        rb.setPower(0);
        lf.setPower(0);
        rf.setPower(0);

        // Turn off RUN_TO_POSITION
        lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

    }
   /** Initializes the TFOD and April Tag processors. **/
    public void initProcessors() {
        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.

                .setModelAssetName(TFOD_MODEL_ASSET)
                .setModelLabels(LABELS)

                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(camera);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);
        builder.addProcessor(tagProcessor);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()


    /** Returns whether a tag with the specified ID is currently detected.
     * @param id ID of tag to detect.
     * @return (boolean) Was the tag detected? **/
    public boolean detectTag(int id){
        int i;
        for (i = 0; i < tagProcessor.getDetections().size(); i++)
        {
            if (tagProcessor.getDetections().get(i).id == id){
                return true;
            }
        }
        return false;
    }

    /** Detects the team prop and returns its X coordinate relative to the camera.
     * (-1 if none is detected)
     * @return (double) The X coordinate of the team prop. **/
    public double detectProp() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (int i = 0; i < 5 && currentRecognitions.size() == 0; i++) {
            sleep(100);
            currentRecognitions = tfod.getRecognitions();
        }
        x = -1;
        double y;
        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        telemetry.update();
        return x;
    }   // end method detectProp()

    /** Uses predefined boundaries to return the spike mark that the team prop is on.
     * @return The spike mark that the team prop is on. **/
    public spike findPos() {
        s(2.5);
        double x = detectProp();
        if (x == -1){
            return spike.left;
        }
        if (x > boundaries[0] && x < boundaries[1]){
            return spike.middle;
        }
        else if (x >= boundaries[1]){
            return spike.right;
        } else if (x <= boundaries[0]){
            return spike.left;
        }
        return spike.none;
    }

    public int setID(spike location, color teamColor) {
        int ID;
        if (teamColor == color.blue) {
            ID = 0;
        } else {
            ID = 3;
        }
        switch (location) {
            case left:
                ID += 1;
                break;
            case middle:
                ID += 2;
                break;
            case right:
                ID += 3;
                break;
            default:
                break;
        }
        return ID;
    }

    /** Sends an exception message to Driver Station telemetry.
     * @param e The exception. **/
    public void except(Exception e) {
        telemetry.addData("Exception", e);
    }

    /** Sleep a specified number of seconds.
     * @param seconds The amount of seconds to sleep. **/
    public final void s(double seconds) {
        sleep((long) seconds * 1000);
    }

    /** Place the purple pixel. **/
    public void purplePixel() {
        s(2);
        drive(-16);
        if (pos == spike.left) {
            turn(-30);
            drive(-9);
            drive(9);
            turn(30);
        } else if (pos == spike.middle) {
            drive(-10);
            drive(10);
        } else {
            turn(30);
            drive(-9);
            drive(9);
            turn(-30);
        }
        drive(20);
        s(.5);
    }

}
