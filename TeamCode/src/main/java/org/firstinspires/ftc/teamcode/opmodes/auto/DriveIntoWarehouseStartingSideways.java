package org.firstinspires.ftc.teamcode.opmodes.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;

@Autonomous
public class DriveIntoWarehouseStartingSideways extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(this.hardwareMap);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        drive.setWeightedDrivePower(new Pose2d(0.5, 0,0));
        timer.reset();
        while (timer.milliseconds() < 1.5) {
            if (isStopRequested()) return;
        }
        drive.setWeightedDrivePower(new Pose2d(0, 0, 0));
    }
}
