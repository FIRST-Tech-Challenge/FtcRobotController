package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+1+Park, Right Quadrant", group = "Worlds Autos")
public class OnePlusOne_RIGHTQUAD extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 2, Constants.BLUE_SCALAR_ARRAY);
    }
}
