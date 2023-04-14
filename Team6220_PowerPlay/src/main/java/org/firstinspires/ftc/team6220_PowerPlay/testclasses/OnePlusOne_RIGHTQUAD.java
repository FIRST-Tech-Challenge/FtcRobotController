package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "less hopium", group = "Worlds Autos")
public class OnePlusOne_RIGHTQUAD extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 1);
    }
}
