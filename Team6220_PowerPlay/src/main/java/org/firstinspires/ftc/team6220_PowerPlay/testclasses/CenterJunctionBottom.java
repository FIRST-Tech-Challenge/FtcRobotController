package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Autonomous(name = "CenterJunctionBottom")
public class CenterJunctionBottom extends BaseAutonomous {

    public RobotCameraPipeline robotCameraPipeline;
    OpenCvCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        robotCameraPipeline.setRanges(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);

        camera.setPipeline(robotCameraPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(Constants.CAMERA_X, Constants.CAMERA_Y, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        waitForStart();

        centerConeStack(robotCameraPipeline);
    }
}
