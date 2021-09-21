package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="TestTeleOp")

public class TestTeleOp extends OpMode {
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
        double turn = gamepad1.right_stick_x*.5;

        if (turn < 0) {
            robot.m0.setPower(drive);
            robot.m1.setPower(drive+turn);
            robot.m2.setPower(drive);
            robot.m3.setPower(drive+turn);
        }
        else{
            robot.m0.setPower(drive-turn);
            robot.m1.setPower(drive);
            robot.m2.setPower(drive-turn);
            robot.m3.setPower(drive);
        }
    }
}
