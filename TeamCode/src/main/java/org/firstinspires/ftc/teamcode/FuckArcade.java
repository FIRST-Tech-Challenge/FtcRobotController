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
        double drive = gamepad1.left_stick_y*.5;
        double turn = gamepad1.right_stick_x*.5;

        robot.m0.set(drive - turn);
        robot.m1.set(drive + turn);
        robot.m2.set(drive - turn);
        robot.m3.set(drive + turn);
//        robot.m_drive.arcadeDrive(drive, turn);
    }
}
