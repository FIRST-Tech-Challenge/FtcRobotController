package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "CenterRedConeStack", group = "Test")
public class CenterRedConeStack extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeCameras(Constants.LOWER_RED, Constants.UPPER_RED);

        waitForStart();

        centerConeStack(robotCameraPipeline);
    }
}
