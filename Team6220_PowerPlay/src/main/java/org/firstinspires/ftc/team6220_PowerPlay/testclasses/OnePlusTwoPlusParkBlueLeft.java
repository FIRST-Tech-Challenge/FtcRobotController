package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+2+Park, Blue Left", group = "Worlds Autos")
public class OnePlusTwoPlusParkBlueLeft extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 2, Constants.BLUE_SCALAR_ARRAY, false);
    }
}
