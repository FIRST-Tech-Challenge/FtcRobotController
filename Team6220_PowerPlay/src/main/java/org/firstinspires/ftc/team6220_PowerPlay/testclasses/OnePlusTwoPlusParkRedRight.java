package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+2+park, Red Right", group = "Worlds Autos")
public class OnePlusTwoPlusParkRedRight extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 2, Constants.RED_SCALAR_ARRAY, true);
    }
}
