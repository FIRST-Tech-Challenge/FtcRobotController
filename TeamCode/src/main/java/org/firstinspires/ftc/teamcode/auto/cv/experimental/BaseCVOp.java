package org.firstinspires.ftc.teamcode.auto.cv.experimental;

import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.FRAME_HEIGHT;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.FRAME_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.HardwareNew;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="Base")
public class BaseCVOp extends LinearOpMode {
    // store as variable here so we can access the location
    BasePipe detector = new BasePipe(telemetry);
    OpenCvCamera Camera;
    HardwareNew robot = new HardwareNew();


    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        // robot logic...

        // https://github.com/OpenFTC/EasyOpenCV/blob/master/examples/src/main/java/org/openftc/easyopencv/examples/InternalCameraExample.java
        // Initialize the back-facing camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        Camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        Camera.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        Camera.setPipeline(detector);
        // Remember to change the camera rotation
        Camera.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPRIGHT);

        //...

        waitForStart();

        while(opModeIsActive())
        {

        }

        // more robot logic...
    }

}