package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+2+park, Red Left", group = "Worlds Autos")
public class OnePlusTwoPlusParkRedLeft extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.LeftAutos, 2, Constants.BLUE_SCALAR_ARRAY, true);
    }
}
