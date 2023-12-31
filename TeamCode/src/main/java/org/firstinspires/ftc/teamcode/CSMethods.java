package org.firstinspires.ftc.teamcode;

import static java.lang.Math.*;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

import org.firstinspires.ftc.robotcore.external.hardware.camera.*;
import org.firstinspires.ftc.robotcore.external.tfod.*;
import org.firstinspires.ftc.vision.*;
import org.firstinspires.ftc.vision.tfod.*;
import java.util.*;

public abstract class CSMethods extends LinearOpMode {
    public static final boolean USE_WEBCAM = true;
    public TfodProcessor tfod;
    public final ElapsedTime runtime = new ElapsedTime();
    public DcMotorEx lf = null;
    public DcMotorEx lb = null;
    public DcMotorEx rf = null;
    public DcMotorEx rb = null;
    public DcMotorEx carWashMotor = null;
    public IMU imu = null;
    public DcMotorEx pixelLiftingMotor = null;
    public Servo droneServo = null;
    public Servo pixelBackServo = null;
    public Servo pixelFrontServo = null;
    public Servo trayTiltingServo = null;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //*/
    static final double     PI                      = Math.PI;
    static final double     COUNTS_PER_MOTOR_REV    = (double) ((((1+(46.0/17))) * (1+(46.0/11))) * 28) ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double     VELOCITY                = 2000;
    static final double     STRAFE_FRONT_MODIFIER   = 1.3;
    static final double     VEL_MODIFIER            = 1.12485939258;
    static final double     b           = 1.1375;
    static final double     m           = 0.889;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    static final double[] boundaries = {0, 350};
    double carWashPower = 1.0;
    double pos; // Team prop position
    public VisionPortal visionPortal;
    public static String TFOD_MODEL_ASSET = null;

    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "prop",
    };
    IMU.Parameters imuparameters;
    public void setup(boolean isRed) {
        if (isRed) {
            TFOD_MODEL_ASSET = "CSTeamPropRed.tflite";
        } else {
            TFOD_MODEL_ASSET = "CSTeamPropBlue.tflite";
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
        boolean imuinit = imu.initialize(imuparameters);
        if (!imuinit){
            telemetry.addData("IMU","Initialization failed");
        }
        try {
            lf = hardwareMap.get(DcMotorEx.class, "leftFront");
            lb = hardwareMap.get(DcMotorEx.class, "leftBack");
            rf = hardwareMap.get(DcMotorEx.class, "rightFront");
            rb = hardwareMap.get(DcMotorEx.class, "rightBack");
        } catch (Exception e) {except(e);}
        try {carWashMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");}catch (Exception e){except(e);}
        try {pixelLiftingMotor = hardwareMap.get(DcMotorEx.class,"pixelLiftingMotor");}catch (Exception e){except(e);}
        try {droneServo = hardwareMap.get(Servo.class, "droneServo");}catch (Exception e){except(e);}
        try {pixelBackServo = hardwareMap.get(Servo.class,"pixelBackServo");}catch (Exception e){except(e);}
        try {pixelFrontServo = hardwareMap.get(Servo.class, "pixelFrontServo");}catch (Exception e){except(e);}
        try {trayTiltingServo = hardwareMap.get(Servo.class,"trayTiltingServo");}catch (Exception e){except(e);}

        initTfod();

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

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int lbTarget = 0;
        int rbTarget = 0;
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
            lb.setVelocity(VELOCITY * signum(leftInches));
            rb.setVelocity(VELOCITY * signum(leftInches));
            lf.setVelocity(VELOCITY * signum(leftInches));
            rf.setVelocity(VELOCITY * signum(leftInches));

            double inches = signum(leftInches) * (abs(leftInches) + b) / m;

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
    public void turn(double degrees) {
        sleep(100);
        imu.resetYaw();
        double tolerance = 1;
        if (false) { // Boolean determines the method the robot takes to turn x degrees
            encoderDrive(TURN_SPEED, degrees / 7.5, -degrees / 7.5, abs(degrees) / 36);
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
    public void strafe(double inches /*double duration*/) {

        if (opModeIsActive() && lf != null) {
            lb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rb.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            lf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rf.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

            lb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


            runtime.reset();
            lb.setVelocity(VELOCITY);
            rb.setVelocity(-VELOCITY);
            lf.setVelocity(-VELOCITY * STRAFE_FRONT_MODIFIER);
            rf.setVelocity(VELOCITY * STRAFE_FRONT_MODIFIER);

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
    public void drive(double inches) {
        double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        int checks = 1; // Number of times the robot will check its orientation during a single drive movement and correct itself
        for(int i = 0; i < checks; i++) {
            encoderDrive(DRIVE_SPEED, inches / checks, inches / checks, abs(inches) / checks / 4 + 1);
            //turn(startAngle - imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        }
        stopRobot();
    }
    public void ejectPixel() {
        telemetry.addData("Car Wash", "Ejecting Pixel");
        telemetry.update();
        if (carWashMotor != null) {
            int t = (int) runtime.milliseconds() + 1000;
            carWashMotor.setPower(carWashPower);
            while (true) {
                if (!(t > ((int) runtime.milliseconds()))) {
                    break;
                }
            }
            carWashMotor.setPower(0);
        }
    }
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
    public void dropCarWash() {
        drive(15);
        drive(-15);
        sleep(100);
    }
    public void initTfod() {

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

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
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

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public double detectProp() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        for (int i = 0; i < 5 && currentRecognitions.size() == 0; i++) {
            sleep(100);
            currentRecognitions = tfod.getRecognitions();
        }
        double x = -1;
        double y = -1;
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

    public void findPos() {
        double x = detectProp();
        if (x > boundaries[0] && x < boundaries[1]){
            pos = 2; // Middle
        }
        else if (x > boundaries[1]){
            pos = 3; // Right
        } else {
            pos = 1; // Left
        }
        telemetry.addData("Position", pos);
        telemetry.update();
    }

    public void except(Exception e) {
        telemetry.addData("Exception", e);
        telemetry.update();
    }

}
