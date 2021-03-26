package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.TreeMap;

@Autonomous

public class finalAutonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_QUAD = "four";
    private static final String LABEL_SINGLE = "one";

    private static final String VUFORIA_KEY = ftcsecrets.secrets.VUFORIA_KEY;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tensorFlowObjDetector;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooter;
    private DcMotor belt;

    private boolean shouldShoot = true;
    private boolean shouldDrive = false;
    private boolean shouldDetectRings = true;
    private boolean ringDetectTestMode = false;

    //code to play once the OpMode is active
    public void runOpMode() {

        initDriveMotors();
        initShootingMotors();

        initVuforia();
        initTensorFlowObjDetector();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        shoot(0.75);

        int zone;
        if (opModeIsActive()) {
            do {
                //zone = determineZone();
                zone = calculateZone();
                switch (zone) {
                    case 0:
                        //move to zone A
                        break;
                    case 1:
                        //move to zone B
                        break;
                    case 2:
                        //move to zone C
                        break;
                    default:
                        //telemetry.addData("Status:", "Invalid number of rings");
                        break;
                }
            } while (opModeIsActive() && ringDetectTestMode == true);
            if (ringDetectTestMode){
                while (opModeIsActive()){
                    telemetry.update();
                }
            }
        }

        if (tensorFlowObjDetector != null) {
            tensorFlowObjDetector.shutdown();
        }
        move(1, 1750);
    }

    public void move(double speed, int time) {
        if (shouldDrive) {
            frontLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setPower(speed);
            backRight.setPower(speed);
            sleep(time);
        }
    }

    public void strafeLeft(int time) {
        if (shouldDrive) {
            frontLeft.setPower(-0.5);
            backLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backRight.setPower(-0.5);
            sleep(time);
        }
    }

    public void strafeRight(int time) {
        if (shouldDrive) {
            frontLeft.setPower(0.5);
            backLeft.setPower(-0.5);
            frontRight.setPower(-0.5);
            backRight.setPower(0.5);
            sleep(time);
        }
    }

    public void shoot(double power) {
        if (shouldShoot) {
            shooter.setPower(power);
            sleep(1500);
            belt.setPower(.5);
            sleep(4000);
            shooter.setPower(0);
            belt.setPower(0);
        }
    }

    public int calculateZone() {
        int samples = 50;
        telemetry.addData(">", "Taking " + samples + " samples...");
        telemetry.update();

        Integer[] zones = {0, 0, 0, 0};

        for (int i=0; i<samples; i++) {
            int zone = determineZone();
            switch(zone) {
                case 0: zones[0]++;
                        break;
                case 1: zones[1]++;
                    break;
                case 2: zones[2]++;
                    break;
                default: zones[3]++;
                    break;
            }
            sleep(200);
        }

        int largestIndex = 0;
        for(int i = 0; i < zones.length; i++ ) {
            telemetry.addData("zone" + i, zones[i]);
            if (zones[i] > zones[largestIndex]) {
                largestIndex = i;
            }
        }

        telemetry.addData(">", "Calculated zone " + largestIndex + " as highest value.");
        telemetry.update();

        if (largestIndex == 3) {
            return -1;
        } else {
            return largestIndex;
        }
    }

    public int determineZone() {
        int zone = -1;

        if (shouldDetectRings && tensorFlowObjDetector != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tensorFlowObjDetector.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0) {
                    telemetry.addData("TFOD", "No items detected.");
                    telemetry.addData("Target Zone", "A");
                    zone = 0;
                } else {
                    // list is not empty.
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals(LABEL_SINGLE)) {
                            telemetry.addData("Target Zone", "B");
                            zone = 1;
                        } else if (recognition.getLabel().equals(LABEL_QUAD)) {
                            telemetry.addData("Target Zone", "C");
                            zone = 2;
                        } else {
                            telemetry.addData("Target Zone", "UNKNOWN");
                            zone = -1;
                        }
                    }
                }

                telemetry.update();
            }

        }

        return zone;
    }

    private void initDriveMotors(){
        if (shouldDrive) {
            frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
            frontRight = hardwareMap.get(DcMotor.class, "frontRight");
            backLeft = hardwareMap.get(DcMotor.class, "backLeft");
            backRight = hardwareMap.get(DcMotor.class, "backRight");

            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }

    private void initShootingMotors(){
        if (shouldShoot) {
            shooter = hardwareMap.get(DcMotor.class, "buzz");
            belt = hardwareMap.get(DcMotor.class, "belt");

            shooter.setDirection(DcMotorSimple.Direction.FORWARD);
            belt.setDirection(DcMotorSimple.Direction.REVERSE);
        }
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

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTensorFlowObjDetector() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tensorFlowObjDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tensorFlowObjDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_QUAD, LABEL_SINGLE);
        tensorFlowObjDetector.activate();
        tensorFlowObjDetector.setZoom(2, 2.5);
    }
}
