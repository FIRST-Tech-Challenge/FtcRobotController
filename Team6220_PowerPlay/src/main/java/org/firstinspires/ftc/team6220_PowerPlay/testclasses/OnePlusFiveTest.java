package org.firstinspires.ftc.team6220_PowerPlay.testclasses;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.team6220_PowerPlay.BaseAutonomous;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Disabled
@Autonomous(name = "hopium", group = "Worlds Autos")
public class OnePlusFiveTest extends OnePlusNAutonFramework {
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 5, Constants.BLUE_SCALAR_ARRAY, false);
    }
}
