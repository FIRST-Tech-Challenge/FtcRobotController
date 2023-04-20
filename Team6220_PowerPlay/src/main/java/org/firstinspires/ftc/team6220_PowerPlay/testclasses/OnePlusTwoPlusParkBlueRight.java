package org.firstinspires.ftc.team6220_PowerPlay.testclasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.internal.system.CloseableOnFinalize;
import org.firstinspires.ftc.team6220_PowerPlay.Constants;

@Autonomous(name = "1+2+park, Blue Right", group = "Worlds Autos")
public class OnePlusTwoPlusParkBlueRight extends OnePlusNAutonFramework{
    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos, 2, Constants.BLUE_SCALAR_ARRAY, false);
    }
}
