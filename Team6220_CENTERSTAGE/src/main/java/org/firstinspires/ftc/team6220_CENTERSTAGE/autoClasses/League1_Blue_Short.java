package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="League1_Blue_Short", group ="amogus2")
public class League1_Blue_Short extends League1_AutoFramework {

    @Override
    public void runOpMode() throws InterruptedException {

        doAutoDriveInches(AutoAlliance.BLUE, AutoType.SHORT);

    }
}