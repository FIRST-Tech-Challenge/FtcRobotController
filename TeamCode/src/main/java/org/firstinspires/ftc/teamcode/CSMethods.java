package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;
import static java.lang.Math.min;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import java.util.List;


public abstract class CSMethods extends LinearOpMode {
    public static final boolean USE_WEBCAM = true;
    public TfodProcessor tfod;
    public final ElapsedTime runtime = new ElapsedTime();
    public DcMotor lf = null;
    public DcMotor lb = null;
    public DcMotor rf = null;
    public DcMotor rb = null;
    public DcMotor carWashMotor = null;
    public IMU imu = null;
    public DcMotor pixelLiftingMotor = null;
    public Servo droneServo = null;
    /*
     - Calculate the COUNTS_PER_INCH for your specific drive train.
     - Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
     - For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
     - For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
     - This is gearing DOWN for less speed and more torque.
     - For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //*/
    static final double     PI                      = 3.141592653589793238462643383279502884197169399; // 47 digits of pi
    static final double     COUNTS_PER_MOTOR_REV    = (double) ((((1+(46.0/17))) * (1+(46.0/11))) * 28) ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing
    static final double     WHEEL_DIAMETER_INCHES   = 3.77953 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * PI);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    double carWashPower = 1.0;
    public VisionPortal visionPortal;
    public static String TFOD_MODEL_ASSET = null;

    // Define the labels recognized in the model for TFOD (must be in training order!)
    public static final String[] LABELS = {
            "prop",
    };

    public void setup(boolean isRed) {
        if (isRed) {
            TFOD_MODEL_ASSET = "CSTeamPropRed.tflite";
        }
        else{
            TFOD_MODEL_ASSET = "CSTeamPropBlue.tflite";
        }

        lf = hardwareMap.get(DcMotor.class, "leftFront");
        lb = hardwareMap.get(DcMotor.class, "leftBack");
        rf = hardwareMap.get(DcMotor.class, "rightFront");
        rb = hardwareMap.get(DcMotor.class, "rightBack");
        carWashMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        imu = hardwareMap.get(IMU.class, "imu");
        pixelLiftingMotor = hardwareMap.get(DcMotor.class,"pixelLiftingMotor");
        droneServo = hardwareMap.get(Servo.class, "droneServo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        pixelLiftingMotor.setTargetPosition(pixelLiftingMotor.getCurrentPosition());
        pixelLiftingMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        initTfod();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newLeftBackTarget;
        int newRightBackTarget;
        int newLeftFrontTarget;
        int newRightFrontTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftBackTarget = lb.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightBackTarget = rb.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newLeftFrontTarget = lf.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightFrontTarget = rf.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);

            lb.setTargetPosition(newLeftBackTarget);
            rb.setTargetPosition(newRightBackTarget);
            lf.setTargetPosition(newLeftFrontTarget);
            rf.setTargetPosition(newRightFrontTarget);

            // Turn On RUN_TO_POSITION
            lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            lb.setPower(abs(speed));
            rb.setPower(abs(speed));
            lf.setPower(abs(speed));
            rf.setPower(abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (lb.isBusy() && rb.isBusy() && lf.isBusy() && rf.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Angle", imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                telemetry.addData("Running to",  " %7d :%7d", newLeftFrontTarget,  newRightFrontTarget);
                telemetry.addData("Currently at",  " at %7d :%7d", lf.getCurrentPosition(), rf.getCurrentPosition());
                telemetry.update();
            }

            //  stopRobot();

            // Turn off RUN_TO_POSITION
            // Note: Following code is technically redundant since called in stopRobot(), but the function
            // may be changed, so do not delete.
            lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //sleep(250);   // optional pause after each move.
        }
    }
    public void turn(double degrees) {
        double TURN_ACCURACY = 0.75;
        if (false) { // Boolean determines the method the robot takes to turn x degrees
            encoderDrive(TURN_SPEED, degrees / 7.5, -degrees / 7.5, abs(degrees) / 36);
            stopRobot();
        } else {
            degrees *= -1;
            double startAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            double currentAngle;
            double initialGoalAngle = startAngle + degrees;
            double correctedGoalAngle = initialGoalAngle;
            double difference;
            double turnModifier;
            double turnPower;
            if (abs(initialGoalAngle) > 180) {
                correctedGoalAngle -= abs(initialGoalAngle) / initialGoalAngle * 360;
            }
            while (opModeIsActive() && (imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - correctedGoalAngle > TURN_ACCURACY || imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle - correctedGoalAngle < -TURN_ACCURACY)) {
                currentAngle = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
                difference = min(abs(initialGoalAngle - currentAngle), abs(correctedGoalAngle - currentAngle));
                turnModifier = Math.min(1, (difference + 5) / 30);
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
        int t = (int) runtime.milliseconds() + 1000;
        carWashMotor.setPower(carWashPower);
        while(true) {
            if (!(t > ((int) runtime.milliseconds()))){
                break;
            }
        }
        carWashMotor.setPower(0);
    }
    public void stopRobot() {
        // Set target position to avoid an error
        lb.setTargetPosition(lb.getCurrentPosition());
        rb.setTargetPosition(rb.getCurrentPosition());
        lf.setTargetPosition(lf.getCurrentPosition());
        rf.setTargetPosition(rf.getCurrentPosition());

        // Turn On RUN_TO_POSITION
        lb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lf.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
    public List<Recognition> detectProp() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop
        return currentRecognitions;
    }   // end method detectProp()

}
