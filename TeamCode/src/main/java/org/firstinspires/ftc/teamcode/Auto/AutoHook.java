// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Swerve.Swerve;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.Swerve.wpilib.kinematics.ChassisSpeeds;
import org.firstinspires.ftc.teamcode.Utils;

@Autonomous(name = "Auto Hook Specimen")
public class AutoHook extends LinearOpMode {
    private Swerve drivebase;

    @Override
    public void runOpMode() throws InterruptedException {
        drivebase = new Swerve(this);
        drivebase.initGyro();

        waitForStart();
        ChassisSpeeds speeds = new ChassisSpeeds(4, 0, 0);
        drivebase.alignWheels(this::opModeIsActive);    //TODO: rear right wheel just spins idk why
        sleep(2000);
        driveWithOdo(speeds, 2.5);
    }

    public void driveWithOdo(ChassisSpeeds speeds, double dt) {
        double startTime = Utils.getTimeSeconds();
        while (opModeIsActive()) {
            double currentTime = Utils.getTimeSeconds() - startTime;
            driveForTime(speeds, currentTime / 2);
            var currentPos = drivebase.getPose();
            Rotation2d rotation = new Rotation2d();   //TODO: figure out a way to make this rotation2d not just 0
            var wantedPos = new Pose2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, rotation);   //TODO: figure out predetermined variables of the positions that we want
            if (currentPos != wantedPos) {
                var movement = currentPos.minus(wantedPos);  //TODO: figure out if this actually works with translating the robot to move where it's supposed to
                ChassisSpeeds newSpeeds = new ChassisSpeeds(movement.getX(), movement.getY(), movement.getRotation().getRadians());
                driveForTime(newSpeeds, currentTime / 2);
            } else
                return;
            if (Utils.getTimeSeconds() - startTime >= dt) {
                return;
            }
        }
    }

    private void driveForTime(ChassisSpeeds speeds, double time) {
        double startTime = Utils.getTimeSeconds();
        double lastTime = startTime;
        while (opModeIsActive()) {
            double currentTime = Utils.getTimeSeconds();
            drivebase.drive(speeds, currentTime - lastTime);
            if (Utils.getTimeSeconds() - startTime >= time) {
                return;
            }
            lastTime = currentTime;
        }
    }

    private void hookClip() {

    }
}
