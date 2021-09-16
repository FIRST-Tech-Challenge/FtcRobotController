package teamcode.test.CVNew;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvTracker;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.common.AbstractOpMode;

public class CVPositioning extends AbstractOpMode {


    private OpenCvWebcam webcam;

    public void onInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        GamePieceTracker tracker;
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
        waitForStart();
    }

    @Override
    protected void onStart() {

    }

    @Override
    protected void onStop() {

    }


    public class GamePieceTracker extends OpenCvTracker{



        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }
    //Vuforia processed through the OpenCV engine
}
