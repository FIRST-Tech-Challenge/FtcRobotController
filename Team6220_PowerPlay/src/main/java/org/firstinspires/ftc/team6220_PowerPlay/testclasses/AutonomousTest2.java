package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.SwitchableCamera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

import java.util.List;

//@Disabled
@TeleOp(name = "AutonomousTest2", group = "Test")
public class AutonomousTest2 extends LinearOpMode {
    private VuforiaLocalizer vuforia;
    private WebcamName robotCamera, grabberCamera;
    private SwitchableCamera camera;
    private boolean oldLeftBumper;
    private boolean oldRightBumper;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        initializeVuforia();
        initializeTensorFlow();

        tfod.activate();
        tfod.setZoom(1.0, 16.0 / 9.0);

        waitForStart();

        while (opModeIsActive()) {
            doCameraSwitching();

            List<Recognition> recognitions = tfod.getRecognitions();
            telemetry.addData("number of objects detected", recognitions.size());

            for (Recognition recognition : recognitions) {
                double col = (recognition.getLeft() + recognition.getRight()) * 0.5;
                double row = (recognition.getTop() + recognition.getBottom()) * 0.5;
                double width = recognition.getWidth();
                double height = recognition.getHeight();

                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);
            }

            telemetry.update();
        }
    }

    // initialize the Vuforia localization engine
    private void initializeVuforia() {
        // configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = Constants.VUFORIA_KEY;

        robotCamera = hardwareMap.get(WebcamName.class, "RobotCamera");
        grabberCamera = hardwareMap.get(WebcamName.class, "GrabberCamera");
        parameters.cameraName = ClassFactory.getInstance().getCameraManager().nameForSwitchableCamera(robotCamera, grabberCamera);

        // instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // set the active camera to the robot camera
        camera = (SwitchableCamera) vuforia.getCamera();
        camera.setActiveCamera(robotCamera);
    }

    // initialize the TensorFlow object detection engine
    private void initializeTensorFlow() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        //tfod.loadModelFromAsset(Constants.TENSORFLOW_MODEL_ASSET, Constants.TENSORFLOW_LABELS);
    }

    private void doCameraSwitching() {
        boolean newLeftBumper = gamepad1.left_bumper;
        boolean newRightBumper = gamepad1.right_bumper;

        if (newLeftBumper && !oldLeftBumper) {
            camera.setActiveCamera(robotCamera);
        } else if (newRightBumper && !oldRightBumper) {
            camera.setActiveCamera(grabberCamera);
        }

        oldLeftBumper = newLeftBumper;
        oldRightBumper = newRightBumper;

        if (camera.getActiveCamera().equals(robotCamera)) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addData("Press RightBumper", "to switch to Webcam 2");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
        }
    }
}
