package org.firstinspires.ftc.teamcode.practice;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisims.MecanumDrive;

@TeleOp
@Disabled
public class MechenumFieldOrientated extends OpMode {
    MecanumDrive drive;
    double forward, strafe, rotate;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap);
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveFieldRelative(forward, strafe, rotate);

        drive.driveFieldRelative(forward,strafe,rotate);
    }
}
