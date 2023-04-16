package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+1+park, Left Quadrant", group = "Worlds Autos")
public class OnePlusOne_LEFTQUAD extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 1, Constants.BLUE_SCALAR_ARRAY);
    }
}
