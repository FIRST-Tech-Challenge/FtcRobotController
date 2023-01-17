package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.RobotCameraPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
@Autonomous(name = "CenterBlueConeStack", group = "Test")
public class CenterBlueConeStack extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeCameras(Constants.LOWER_BLUE, Constants.UPPER_BLUE);

        waitForStart();

        centerConeStack(robotCameraPipeline);
    }
}
