package org.firstinspires.ftc.team6220_PowerPlay.competition;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_PowerPlay.AutoFramework;

@Autonomous(name = "BlueRight")
public class BlueRight extends AutoFramework {

    @Override
    public void runOpMode() throws InterruptedException {
        runAuto(AutoState.RightAutos);
    }

}
