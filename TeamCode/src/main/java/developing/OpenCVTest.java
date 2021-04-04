
package developing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@TeleOp
public class OpenCVTest extends LinearOpMode
{
    OpenCvInternalCamera2 phoneCam;

    TerraCV terraCV = new TerraCV();

    @Override
    public void runOpMode()
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(terraCV);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
//
//        phoneCam.setFlashlightEnabled(true);

//        phoneCam.setSensorFps(30);

        waitForStart();

        while (opModeIsActive())
        {

            telemetry.addData("Frame Count", phoneCam.getFrameCount());
            telemetry.addData("FPS", String.format("%.2f", phoneCam.getFps()));
            telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            telemetry.addData("RingNum", terraCV.ringNum);
            telemetry.update();

            if(gamepad1.a)
            {
                phoneCam.stopStreaming();
                //phoneCam.closeCameraDevice();
            }

            sleep(100);
        }

    }

}