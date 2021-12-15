//TODO: Test tflite model.
//TODO: Change movement values.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

@Autonomous

public class autonomous extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy.tflite";
    private static final String LABEL_LEFT = "left";
    private static final String LABEL_MIDDLE = "middle";
    private static final String LABEL_RIGHT = "right";

    private static final String VUFORIA_KEY = ftcsecrets.secrets.autonomousKey;

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tensorFlowObjDetector;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private boolean shouldShoot = false;
    private boolean shouldDrive = false;
    private boolean shouldDetectRings = true;
    private boolean ringDetectTestMode = false;

    //code to play once the OpMode is active
    public void runOpMode() {

        initDriveMotors();

        initVuforia();
        initTensorFlowObjDetector();

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        int zone;
        if (opModeIsActive()) {
            do {
                zone = calculateZone();
                switch (zone) {
                    case 0:
                        break;
                    case 1:
                        break;
                    case 2:
                        break;
                    default:
                        break;
                }
            } while (opModeIsActive() && ringDetectTestMode == true);
        }

        if (tensorFlowObjDetector != null) {
            tensorFlowObjDetector.shutdown();
        }
    }

    public int calculateZone() {
        int samples = 17;
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
            sleep(203);
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
                if (updatedRecognitions.size() >= 0) {
                    for (Recognition recognition : updatedRecognitions) {

                        // check label to see which target zone to go after.
                        if (recognition.getLabel().equals(LABEL_MIDDLE)) {
                            telemetry.addData("Target Zone", "Top");
                            zone = 1;
                        } else if (recognition.getLabel().equals(LABEL_LEFT)) {
                            telemetry.addData("Target Zone", "Middle");
                            zone = 2;
                        } else if (recognition.getLabel().equals(LABEL_RIGHT)){
                            telemetry.addData("Target Zone", "Bottom");
                            zone = 0;
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

            frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
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
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tensorFlowObjDetector = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tensorFlowObjDetector.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_LEFT, LABEL_MIDDLE, LABEL_RIGHT);
        tensorFlowObjDetector.activate();
        tensorFlowObjDetector.setZoom(1.75, 1.78);
    }
}