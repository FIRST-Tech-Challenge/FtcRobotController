package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//test1
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
//test
import java.util.List;
import java.util.Locale;

@Disabled
@Autonomous(name = "LeftPowerPlaySleeveCone", group = "")
public class LeftPowerPlaySleeveCone extends LinearOpMode {
    //test1
    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public float desiredHeading;

    private PIDController pidRotate;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String[] LABELS = {
            "Bolt",
            "Bulbs",
            "Panels"
    };
    private static final String VUFORIA_KEY =
            "AVXWcGz/////AAABmZfYj2wlVElmo2nUkerrNGhEBBg+g8Gq1KY3/lN0SEBYx7HyMslyrHttOZoGtwRt7db9nfvCiG0TBEp7V/+hojHXCorf1CEvmJWWka9nFfAbOuyl1tU/IwdgHIvSuW6rbJY2UmMWXfjryO3t9nNtRqX004LcE8O2zkKdBTw0xdqq4dr9zeA9gX0uayps7t0TRmiToWRjGUs9tQB3BDmSinXxEnElq+z3SMJGcn5Aj44iEB7uy/wuB8cGCR6GfOpDRYqn/R8wwD757NucR5LXA48rulTdthGIuHoEjud1QzyQOv4BpaODj9Oi0TMuBmBzhFJMwWzyZ4lKVyOCbf3uCRia7Q+HO+LbFbghNIGIIzZC";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private int resultROI=2;

    private  boolean done = false;


    @Override
    public void runOpMode() throws InterruptedException {

        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");

        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        LF.setDirection(DcMotor.Direction.REVERSE);  // motor direction set for mecanum wheels with mitre gears
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);



        // IMU initialization
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        pidRotate = new PIDController(.003, .00003, 0);


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Set up our telemetry dashboard
        composeTelemetry();  // need to add this method at end of code

        desiredHeading = getHeading();

        moveUtils.initialize(LF, RF, LB, RB, imu, desiredHeading, pidRotate);
        moveUtils.resetEncoders();

        actuatorUtils.initializeActuator(arm, gripper);


        Long startTime = System.currentTimeMillis();
        Long currTime = startTime;

        initVuforia();
        initTfod();

        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.5, 16.0 / 9.0);
        }
        actuatorUtils.gripperClose(false);


        waitForStart();
        currTime=System.currentTimeMillis();
        startTime=currTime;
        if (tfod != null && resultROI == 2) {

            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            while (!done && opModeIsActive()) {
                if((currTime - startTime) < 3000){
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Objects Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() != 0) {
                            done = true;
                        }
                        // step through the list of recognitions and display image position/size information for each one
                        // Note: "Image number" refers to the randomized image orientation/number
                        for (Recognition recognition : updatedRecognitions) {
                            double col = (recognition.getLeft() + recognition.getRight()) / 2;
                            double row = (recognition.getTop() + recognition.getBottom()) / 2;
                            double width = Math.abs(recognition.getRight() - recognition.getLeft());
                            double height = Math.abs(recognition.getTop() - recognition.getBottom());

                            telemetry.addData("", " ");
                            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                            telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                            telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);

                            String imageCheck = recognition.getLabel();
                            if (imageCheck.equals("Bolt")) {
                                resultROI = 1;
                            } else if (imageCheck.equals("Bulbs")) {
                                resultROI = 2;
                            } else if (imageCheck.equals("Panels")) {
                                resultROI = 3;
                            } else {
                                resultROI = 2;
                            }
                            telemetry.addData("ResultROI", resultROI);

                        }
                        telemetry.update();
                    }
                    currTime = System.currentTimeMillis();}

                else {
                    vuforia.close();
                    done=true;
                    resultROI=2;
                }
            }

        }
        telemetry.update();
        done = false;
        //lift arm up
        actuatorUtils.armPole(4);
        while (((currTime - startTime) < 30000)&& !done && opModeIsActive()) {

            switch (resultROI) {
                case 1:
                    // Far left
                    beginAuto();
                    moveUtils.goStraight(-40,MAX_SPEED,MIN_SPEED,ACCEL);
                    telemetry.addData("Strafing", "case 1");
                    telemetry.update();
                    done=true;
                    break;
                case 2:
                    // Middle
                    telemetry.addData("Executing", "case 2");
                    telemetry.update();
                    beginAuto();
                    moveUtils.strafeBuddy(8);
                    moveUtils.goStraight(-16,MAX_SPEED,MIN_SPEED,ACCEL);
                    moveUtils.strafeBuddy(-4);
                    done=true;
                    break;
                case 3:
                    // Far right
                    telemetry.addData("Executing", "case 3");
                    telemetry.update();
                    beginAuto();
                    actuatorUtils.armPole(4);
                    done=true;
                    break;
            }

            currTime = System.currentTimeMillis();

        }
    }
    private void beginAuto() throws InterruptedException {
        moveUtils.goStraight(2,MAX_SPEED,MIN_SPEED,ACCEL);
        moveUtils.turnCW(90);
        moveUtils.goStraight(24,MAX_SPEED,MIN_SPEED,ACCEL);
        moveUtils.turnCCW(90);
        moveUtils.goStraight(22,MAX_SPEED,MIN_SPEED,ACCEL);
        moveUtils.turnCW(45);
        actuatorUtils.armPole(3);
        moveUtils.goStraight(9,MAX_SPEED,MIN_SPEED,ACCEL);
        actuatorUtils.gripperOpen(true);
        moveUtils.goStraight(-5,MAX_SPEED,MIN_SPEED,ACCEL);
        moveUtils.turnCW(45);

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                // Acquiring the angles is relatively expensive; we don't want
                // to do that in each of the three items that need that info, as that's
                // three times the necessary expense.
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override
                    public String value() {
                        return imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override
                    public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public float getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC,
                AxesOrder.ZYX,
                DEGREES);
        return angles.firstAngle;
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.75f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);
    }

}
