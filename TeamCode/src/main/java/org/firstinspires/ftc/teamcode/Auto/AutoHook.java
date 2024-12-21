// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Swerve.Swerve;
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
        ChassisSpeeds speeds = new ChassisSpeeds(1, -.2, 0);
        drivebase.alignWheels(this::opModeIsActive);
        driveForTime(speeds, 2);
    }

    public void driveWithOdo(double xInput, double yInput, double yawInput,ChassisSpeeds speeds){
        while(opModeIsActive()) {
            drivebase.drive(speeds,0.1);
            for (int i = 0; i < 4; i++) {
                var currentPos = drivebase.modules[i].get;
                var wantedPos = 0;
                if(currentPos != wantedPos){
                    currentPos.minus(wantedPos);
                    drivebase.drive(currentPos,.75);
                }
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

    private void hookClip(){

    }
}
