package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;
import org.firstinspires.ftc.team6220_PowerPlay.testclasses.OnePlusNAutonFramework;

@Autonomous(name = "Red Left")
public class OnePlusTwoPlusParkRedLeft extends OnePlusNAutonFramework {
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 2, Constants.BLUE_SCALAR_ARRAY, true);
    }
}
