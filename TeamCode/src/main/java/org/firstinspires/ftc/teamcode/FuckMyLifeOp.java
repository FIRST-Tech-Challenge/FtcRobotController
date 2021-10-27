package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="FuckMyLifeOp")

public class FuckMyLifeOp extends OpMode {
    Hardware robot = new Hardware();
    MecanumDrive drive;
    @Override
    public void init() {
        robot.init(hardwareMap);
        drive = new MecanumDrive(robot.m0, robot.m1, robot.m2, robot.m3);
    }
    @Override
    public void loop() {
        drive.driveRobotCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
