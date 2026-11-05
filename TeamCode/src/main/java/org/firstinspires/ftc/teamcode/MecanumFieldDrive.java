package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "32226")
public class MecanumFieldDrive extends OpMode {


    MercanDrive drive = new MercanDrive();
    double forward, strafe, rotate;


    @Override
    public void init() {
        drive.init(hardwareMap);
    }
    @Override
    public void loop(){
        forward = gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;


        drive.driveFieldRelative(forward,strafe,rotate);


    }
}

