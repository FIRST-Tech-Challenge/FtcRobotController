package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import androidx.annotation.Nullable;

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
    private static final boolean USE_WEBCAM = true;
    private TfodProcessor tfod;
    private final ElapsedTime runtime = new ElapsedTime();
    // All non-primitive datatypes initialize to null on default.
    public DcMotorEx lf, lb, rf, rb, carWashMotor, pixelLiftingMotor;
    public Servo droneServo, pixelBackServo, pixelFrontServo, trayTiltingServo;
    private WebcamName camera;
    public TouchSensor touchSensor;
    private IMU imu;
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
    static       double     velocity                = 2000;
    static final double     TILE_LENGTH             = 23.25;
    static final double     STRAFE_FRONT_MODIFIER   = 1.3;
    //static final double     VEL_MODIFIER            = 1.12485939258;
    static final double     B                       = 1.1375;
    static final double     M                       = 0.889;
    static final double     TURN_SPEED              = 0.5;
    static final boolean    TURN_TYPE               = false;
    public final double[]   BOUNDARIES              = {0, 350};
    double                  carWashPower            = 1.0;
    spike pos; // Team prop position
    public double x;
    public VisionPortal visionPortal;
    public static String tfodModelName;
    private AprilTagProcessor tagProcessor;
    private final int WAIT_TIME = 500;

    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "prop",
    };
    IMU.Parameters imuParameters;

    /** Color options for the team prop. Options: r, b, n **/
    public enum color {
        r, b, n
    }

    /** Directions. Options: l, r, f, b **/
    public enum dir {
        l, r, f, b
    }

    /** Spike mark positions for the team prop. Options: l, m, r, n **/
    public enum spike {
        l, m, r, n
    }

    /** Initializes all hardware devices on the robot.
     * @param teamColor The color of the team prop.
     * @param useCam Should the camera be initialized? **/
    public void setup(color teamColor, boolean useCam) {
        if (teamColor == color.r) {
            tfodModelName = "Prop_Red.tflite";
        } else if (teamColor == color.b){
            tfodModelName = "Prop_Blue.tflite";
        }
        else if (useCam){
            telemetry.addData("", "Team color not specified, will not use team prop detection!");
            useCam = false;
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        if (!imu.initialize(imuParameters)){
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
            setup(color.r, true);
        }
        else {
            setup(color.b, true);
        }
    }

    /** Drives using encoder velocity. An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.**/
    private void encoderDrive(double inches, dir direction) {
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
            if (inches != 0) {
                runtime.reset();
                lb.setVelocity(velocity * signum(inches));
                rb.setVelocity(velocity * signum(inches));
                lf.setVelocity(velocity * signum(inches));
                rf.setVelocity(velocity * signum(inches));
            }
            else if (direction == dir.f){
                lb.setVelocity(velocity);
                rb.setVelocity(velocity);
                lf.setVelocity(velocity);
                rf.setVelocity(velocity);
            }
            else if (direction == dir.b){
                lb.setVelocity(-velocity);
                rb.setVelocity(-velocity);
                lf.setVelocity(-velocity);
                rf.setVelocity(-velocity);
            }

            if (inches != 0) {
                inches = signum(inches) * (abs(inches) + B) / M;
            }

            double duration = abs(inches * COUNTS_PER_INCH / velocity);

            while (opModeIsActive() && (runtime.seconds() < duration) && inches != 0) {
                // Display it for the driver.
                telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Running to",  " %7d :%7d", lfTarget,  rfTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());
                telemetry.update();
            }
            if (inches != 0) {
              stopRobot();
            }
        }
    }

    /** Drives using encoder velocity. An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive.**/
    private void encoderDrive(double inches){
        encoderDrive(inches, dir.f);
    }

    /** Turns the robot a specified number of degrees. Positive values turn right,
     * negative values turn left. A degrees value of zero will cause the robot to turn until manually stopped.
     * @param degrees The amount of degrees to turn.
     * @param direction (opt.) Direction to turn if degrees is zero.
     */
    public void turn(double degrees, dir direction) {
        double direct = 0;
        if (direction == dir.l) {
            direct = -1;
        } else if (direction == dir.r){
            direct = 1;
        }
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
            while (opModeIsActive() && (difference > tolerance) && degrees != 0) {
                currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                difference = min(abs(initialGoalAngle - currentAngle), abs(correctedGoalAngle - currentAngle));
                turnModifier = min(1, (difference + 3) / 30);
                turnPower = degrees / abs(degrees) * TURN_SPEED * turnModifier * direct;
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
            sleep(WAIT_TIME);
        }
    }

    /** Turns the robot a specified number of degrees. Positive values turn right,
     * negative values turn left. A degrees value of zero will cause the robot to turn until manually stopped.
     * @param degrees The amount of degrees to turn.
     */
    public void turn(double degrees){
        turn(degrees, dir.r);
    }

    /** Strafes left or right for a specified number of inches. An inches value of zero will cause the
     * robot to strafe until manually stopped.
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

            double d = 0;

            if (direction == dir.r) {
                d = -1;
            } else if (direction == dir.l){
                d = 1;
            }

            runtime.reset();
            lb.setVelocity(velocity * d);
            rb.setVelocity(-velocity * d);
            lf.setVelocity(-velocity * STRAFE_FRONT_MODIFIER * d);
            rf.setVelocity(velocity * STRAFE_FRONT_MODIFIER * d);
            if (inches != 0) {
                inches = (abs(inches) + 1.0125) / 0.7155;
            }

            double duration = abs(inches * COUNTS_PER_INCH / velocity);

            while (opModeIsActive() && (runtime.seconds() < duration) && inches != 0) {
                telemetry.addData("Strafing until",  duration + " seconds");
                telemetry.addData("Currently at",  runtime.seconds() + " seconds");
                telemetry.update();
            }
            if (inches != 0) {
                stopRobot();
            }

        }
        sleep(WAIT_TIME);
    }

    public void strafe(double inches){
        strafe(inches, dir.l);
    }

    /** Changes the velocity.
     * @param velocity New velocity value.
     */
    public void setSpeed(double velocity) {
        CSBase.velocity = velocity;
    }

    /** Drives the specified number of inches. Negative values will drive backwards.
     * An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive.
     * @param direction (opt.) Direction to drive if inches is zero.**/
    public void drive(double inches, dir direction) {
        //double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        if (inches != 0) {
            for (int i = 0; i < checks; i++) {
                encoderDrive(inches / checks, direction);
                //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            }
            stopRobot();
        } else {
            encoderDrive(0,direction);
        }
    }

    /** Drives the specified number of inches. Negative values will drive backwards.
     * An inches value of zero will cause the robot to drive until manually stopped.
     * @param inches Amount of inches to drive. **/
    public void drive(double inches){
        drive(inches, dir.f);
    }

    /** Converts an amount of tiles on the game board to an amount of inches.
     * @param tiles The value of tiles to be converted. **/
    public double tilesToInches(double tiles) {
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
        if (lb != null) {
            lb.setPower(0);
            rb.setPower(0);
            lf.setPower(0);
            rf.setPower(0);
            lb.setVelocity(0);
            rb.setVelocity(0);
            lf.setVelocity(0);
            rf.setVelocity(0);

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

            // Turn off RUN_TO_POSITION
            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            sleep(WAIT_TIME);
        }
    }
   /** Initializes the TFOD and April Tag processors. **/
    private void initProcessors() {

        tfod = new TfodProcessor.Builder()

                .setModelAssetName(tfodModelName)
                .setModelLabels(LABELS)

                .build();
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();

        if (USE_WEBCAM) {
            builder.setCamera(camera);
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        builder.enableLiveView(true);

        builder.addProcessor(tfod);
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.75f);


    }


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

    /**
     * Detects AprilTag
     * @param id AprilTag ID
     * @return AprilTag information
     */
    @Nullable
    private AprilTagDetection tagDetections(int id) {
        int i;
        for (i = 0; i < tagProcessor.getDetections().size(); i++)
        {
            if (tagProcessor.getDetections().get(i).id == id){
                return tagProcessor.getDetections().get(i);
            }
        }
        return null;
    }

    /** Attempts to detect AprilTag for a specified number of seconds.
     * @param id AprilTag ID
     * @param seconds Time in seconds to attempt detection
     * @return AprilTag information
     */
    @Nullable
    public AprilTagDetection tagDetections(int id, int seconds) {
        int ms = seconds * 1000;
        AprilTagDetection a = tagDetections(id);
        int t = (int) System.currentTimeMillis();
        while (opModeIsActive() && (a != null &&  t - System.currentTimeMillis() < ms)) {
            a = tagDetections(id);
        }
        return a;
    }

    public void align(int id) {
        AprilTagDetection a = tagDetections(id, 1);
            while (opModeIsActive() && a != null && (abs(a.ftcPose.x) > 3 || abs(a.ftcPose.yaw) > 1)) {
                a = tagDetections(id, 1);
                if (a == null) { return; }
                turn(a.ftcPose.yaw);
                a = tagDetections(id, 1);
                if (a == null) { return; }
                strafe(a.ftcPose.x);
        }
    }

    /** Detects the team prop and returns its X coordinate relative to the camera.
     * (-1 if none is detected)
     * @return (double) The X coordinate of the team prop. **/
    private double detectProp() {
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
        s(2);
        double x = detectProp();
        if (x == -1){
            return spike.l;
        }
        if (x > BOUNDARIES[0] && x < BOUNDARIES[1]){
            return spike.m;
        }
        else if (x >= BOUNDARIES[1]){
            return spike.r;
        } else if (x <= BOUNDARIES[0]){
            return spike.l;
        }
        return spike.n;
    }

    public int setID(spike location, color teamColor) {
        int ID;
        if (teamColor == color.b) {
            ID = 0;
        } else {
            ID = 3;
        }
        switch (location) {
            case l:
                ID += 1;
                break;
            case m:
                ID += 2;
                break;
            case r:
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
        s(.5);
        if (pos == spike.l) {
            turn(-40);
            drive(-8);
            drive(8);
            turn(40);
        } else if (pos == spike.m) {
            drive(-12);
            drive(12);
        } else {
            turn(40);
            drive(-8);
            drive(8);
            turn(-40);
        }
        s(.5);
        turn(-180);
        drive(-25);
    }
}