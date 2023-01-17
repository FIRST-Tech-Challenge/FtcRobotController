package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

//@Disabled
@Autonomous(name = "CenterJunctionBottom", group = "Test")
public class CenterJunctionBottom extends BaseAutonomous {

    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        initializeCameras(Constants.LOWER_YELLOW, Constants.UPPER_YELLOW);

        waitForStart();

        centerConeStack(robotCameraPipeline);
    }
}
