package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.List;
import java.util.Locale;

@Autonomous(name = "RoadRunnerTest", group = "")
public class RoadRunnerTest extends LinearOpMode {
    //test1

    private SampleMecanumDrive drive;
    private Servo gripper = null; //Located on Expansion Hub- Servo port 0
    private DcMotor arm = null;

    static final float MAX_SPEED = 1.0f;
    static final float MIN_SPEED = 0.4f;
    static final int ACCEL = 75;  // Scaling factor used in accel / decel code.  Was 100!
    public float desiredHeading;

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
        drive = new SampleMecanumDrive(hardwareMap);
        arm = hardwareMap.get(DcMotor.class, "arm");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //Reverse the arm direction so it moves in the proper direction
        arm.setDirection(DcMotor.Direction.REVERSE);

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
                    telemetry.addData("Strafing", "case 1");
                    telemetry.update();
                    done=true;
                    break;
                case 2:
                    done=true;
                    break;
                case 3:
                    // Far right
                    telemetry.addData("Executing", "case 3");
                    telemetry.update();
                    done=true;
                    break;
            }

            currTime = System.currentTimeMillis();

        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

    public float getHeading() {
        Orientation angles = drive.getRawExternalHeading();
        return angles;
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

