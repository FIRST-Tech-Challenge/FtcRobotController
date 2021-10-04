package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="FuckMyLifeOp")

public class FuckMyLifeOp extends OpMode {
    Hardware robot = new Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        //left m0,m2
        //right m1,m3
        double drive = gamepad1.left_stick_y*.5;
        double turn = gamepad1.left_stick_x*.5;
        double strafe = gamepad1.right_stick_y*.5;
        double MIN_ANGLE = 0;
        double MAX_ANGLE = 180;
        double degreeRange = robot.servo.getAngle();
        double Position = 69.420;


        robot.servo.setRange(MIN_ANGLE, MAX_ANGLE);

        robot.servo.setPosition(Position);

        robot.mecanum.driveRobotCentric(strafe, drive, turn);
    }
}
