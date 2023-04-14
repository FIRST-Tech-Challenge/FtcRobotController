package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "1+1+park, Left Quadrant", group = "Worlds Autos")
public class OnePlusOne_LEFTQUAD extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 1);
    }
}
