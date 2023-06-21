package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.testclasses.OnePlusNAutonFramework;

@Autonomous(name = "Blue Right")
public class OnePlusTwoPlusParkBlueRight extends OnePlusNAutonFramework {
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 2, Constants.BLUE_SCALAR_ARRAY, false);
    }
}
