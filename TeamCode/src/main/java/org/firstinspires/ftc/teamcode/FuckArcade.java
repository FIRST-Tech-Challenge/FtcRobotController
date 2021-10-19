package org.firstinspires.ftc.teamcode;

import android.drm.DrmStore;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware;

@TeleOp(name="FuckArcade")
public class FuckArcade extends OpMode {

    Hardware robot = new Hardware();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        double drive = gamepad1.right_trigger;
        double turn = gamepad1.left_stick_x;

        robot.mecanum.driveRobotCentric(0,drive,turn);
    }
}
