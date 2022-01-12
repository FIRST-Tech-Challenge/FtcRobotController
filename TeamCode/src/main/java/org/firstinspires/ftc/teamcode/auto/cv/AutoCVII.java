package org.firstinspires.ftc.teamcode.auto.cv;

import static org.firstinspires.ftc.teamcode.common.utils.DriveUtils.encoderDrive;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.FRAME_HEIGHT;
import static org.firstinspires.ftc.teamcode.common.utils.FTCConstants.FRAME_WIDTH;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.BaseNewOpMode;
import org.firstinspires.ftc.teamcode.common.Direction;
import org.firstinspires.ftc.teamcode.config.HardwareNew;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

/**
 * this is the auto for PipelineII, that drives towards out shipping element
 * @author aryansinha
 */
@Autonomous(name="Auto", group="Auto")
public class AutoCVII extends BaseNewOpMode {
    /**
     * {@inheritDoc}
     */
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareNew robot = new HardwareNew();
        CvPipeline detector = new CvPipeline(telemetry);
        Direction direction = detector.getLocation();
        robot.init(hardwareMap);

        OpenCvCamera Camera = initCamera(detector);
        waitForStart();

        while (opModeIsActive())
        {
            double distance = detector.getDistance();
            switch(direction) {
                case LEFT:
                    encoderDrive(this, 0.5,  distance- 1, distance - 1, 5);
                    robot.turnLeft(20, 0.5);
                    break;
                case RIGHT:
                    encoderDrive(this, 0.5,  distance- 1, distance - 1, 5);
                    robot.turnRight(20, 0.5);
                    break;
                case NOT_FOUND:
                    robot.setAllPower(this, 3, 1000);
                    break;
                case IN_FRONT:
                    encoderDrive(this, 0.5,  distance-1, distance - 1, 5);
                    break;
                default:
                    // We want the looping to continue.
            }
        }

        Camera.stopStreaming();
    }

    private OpenCvCamera initCamera(CvPipeline detector) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        // Connect to the camera
        camera.openCameraDevice();
        // Use the SkystoneDetector pipeline
        // processFrame() will be called to process the frame
        camera.setPipeline(detector);
        // Remember to change the camera rotation
        camera.startStreaming(FRAME_WIDTH, FRAME_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
        return camera;
    }
}