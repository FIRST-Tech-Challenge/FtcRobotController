package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "less hopium L", group = "Worlds Autos")
public class OnePlusOne_LEFTQUAD extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 1);
    }
}
