package org.firstinspires.ftc.team6220_CENTERSTAGE.autoClasses;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="League1_Red_Short", group ="amogus2")
public class League1_Red_Short extends League1_AutoFramework {

    @Override
    public void runOpMode() throws InterruptedException {

        doAutoDriveInches(AutoAlliance.RED, AutoType.SHORT);

    }
}