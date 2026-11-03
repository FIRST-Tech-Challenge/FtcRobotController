package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.mecanumDrive;

public class SampleFieldOrientatedMecanumDrive extends OpMode {

    mecanumDrive drive = new mecanumDrive();
    double forward, strafe, rotate;

    public void init(){
        drive.init(hardwareMap);
    }

    public void loop(){
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.driveField(forward,strafe,rotate);
    }
}
