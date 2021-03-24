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

import java.util.List;

@Autonomous

public class finalAutonomous extends LinearOpMode{
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooter;
    private DcMotor belt;

    private static final String TFOD_MODEL_ASSET = "detect.tflite";
    private static final String LABEL_QUAD = "four";
    private static final String LABEL_SINGLE = "one";

    private static ftcsecrets.secrets appSecrets = new ftcsecrets.secrets();

    private static final String VUFORIA_KEY = ftcsecrets.secrets.VUFORIA_KEY;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tensorFlowObjDetector;

    //code to play once the OpMode is active
    public void runOpMode() {

        //hardware mapping
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        shooter = hardwareMap.get(DcMotor.class, "buzz");
        belt = hardwareMap.get(DcMotor.class, "belt");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        belt.setDirection(DcMotorSimple.Direction.REVERSE);

        initVuforia();
        initTensorFlowObjDetector();

        if (tensorFlowObjDetector != null) {
            tensorFlowObjDetector.activate();
            tensorFlowObjDetector.setZoom(2.5, 1.78);
        }

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        waitForStart();

        shooter.setPower(.75);
        sleep(1500);
        belt.setPower(.5);
        sleep(5000);
        shooter.setPower(0);
        belt.setPower(0);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tensorFlowObjDetector != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tensorFlowObjDetector.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 0 ) {
                            // empty list.  no objects recognized.
                            telemetry.addData("TFOD", "No items detected.");
                            telemetry.addData("Target Zone", "A");

                            move(.75, 3000);
                            strafeLeft(500);
                            //drop wobble goal
                            move(-0.75, 500);
                        } else {
                            // list is not empty.
                            // step through the list of recognitions and display boundary info.
                            int i = 0;
                            for (Recognition recognition : updatedRecognitions) {

                                // check label to see which target zone to go after.
                                if (recognition.getLabel().equals(LABEL_SINGLE)) {
                                    telemetry.addData("Target Zone", "B");
                                    move(.75, 3500);
                                    strafeLeft(500);
                                    //drop wobble goal
                                    move(-0.75, 750);
                                } else if (recognition.getLabel().equals(LABEL_QUAD)) {
                                    telemetry.addData("Target Zone", "C");
                                    move(.75, 3750);
                                    strafeLeft(500);
                                    //drop wobble goal
                                    move(-0.75, 1000);
                                } else {
                                    telemetry.addData("Target Zone", "UNKNOWN");
                                }
                            }
                        }

                        telemetry.update();
                    }

                }
            }
        }

        if (tensorFlowObjDetector != null) {
            tensorFlowObjDetector.shutdown();
        }
        move(.5, 3500);
    }
    public void move(double speed, int time){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
        sleep(time);
    }
    public void strafeLeft(int time){
        frontLeft.setPower(-0.5);
        backLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backRight.setPower(-0.5);
        sleep(time);
    }
    public void strafeRight(int time){
        frontLeft.setPower(0.5);
        backLeft.setPower(-0.5);
        frontRight.setPower(-0.5);
        backRight.setPower(0.5);
        sleep(time);
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
    }
}
