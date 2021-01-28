package teamcode.test.AyushVision;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import teamcode.League1.LeagueOnePowerShotAuto;
import teamcode.common.AbstractOpMode;

@TeleOp(name="VisionBasic")
public class VisionDiagnostic extends AbstractOpMode {
    OpenCvWebcam webcam;


    @Override
    protected void onInitialize() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        DisplayPipeline pipeline = new DisplayPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(() -> {
            webcam.startStreaming(320, 240); //specify cam orientation and calibrate the resolution
        });
    }

    @Override
    protected void onStart() {
        while(opModeIsActive());
    }

    @Override
    protected void onStop() {

    }

    private class DisplayPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            return null;
        }
    }
}
