package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
@TeleOp
@Config

public class slideTest extends OpMode {
    nematocyst slide;
    FtcDashboard dash;
    Telemetry t2;
    public static double sP = 0.0009;
    public static double sI = 0.000;
    public static double sD = 0.00;
    public static double angP = 0.00375;
    public static double angI = 0;
    public static double angD = 0.00006;
    public static double angCos = 0.345;
    public static double angExt = 0.000075;
    @Override
    public void init() {
        slide = new nematocyst(this);
        slide.init("pivot", "slide", "wrist", "claw");
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad2.a) {
            slide.goUp(0);
        } else if (gamepad2.x) {
            slide.goGround(13.0);
        } else if (gamepad2.b) {
            slide.goSpecimen(28);
        } else if (gamepad2.y) {
            slide.goOut(36.0);
        } else if (gamepad2.right_trigger > 0.5) {
            slide.goSpecimenDown(2);
        }
        if (gamepad2.dpad_up) {
            slide.wristOut();
        } else if (gamepad2.dpad_down) {
            slide.wristIn();
        } else if (gamepad2.left_bumper) {
            slide.claw.setPosition(slide.claw.getPosition()+0.005);
        } else if (gamepad2.right_bumper) {
            slide.claw.setPosition(slide.claw.getPosition()-0.005);
        } else if (gamepad2.dpad_left) {
            slide.wristDown();
        }

        slide.loop(sP, sI, sD);
        slide.updatePID(angP, angI, angD, angCos, angExt);
        slide.updateSlidePID(sP, sI, sD);
        slide.getTelemetry();
        slide.getTelemetry(t2);

    }
}
