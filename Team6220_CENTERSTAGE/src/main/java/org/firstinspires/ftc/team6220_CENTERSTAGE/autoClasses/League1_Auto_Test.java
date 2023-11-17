package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Config
@Autonomous(name="League1_Auto_Test", group ="amogus2")
public class League1_Auto_Test extends League1_AutoFramework {

    public static double targetHeading = 0.0;
    @Override
    public void runOpMode() throws InterruptedException {

        initHardware();
        waitForStart();

        turnToAngle(targetHeading);

    }
}