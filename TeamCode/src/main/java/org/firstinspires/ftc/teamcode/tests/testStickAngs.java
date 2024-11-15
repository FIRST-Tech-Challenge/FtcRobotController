package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.gamepadToVectors;
@TeleOp
public class testStickAngs extends OpMode {
    gamepadToVectors a;
    FtcDashboard dash;
    Telemetry t2;
    @Override
    public void init() {
        a = new gamepadToVectors();
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
//        double[] vector = a.getCombinedVector(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepadToVectors.Wheel.values()[0]);
//        double angle = Math.toDegrees(Math.atan2(vector[0], vector[1])) + 180;
//        telemetry.addData("angle", angle);
//        t2.addData("ang", angle);
//        t2.update();
//        telemetry.update();
    }
}
