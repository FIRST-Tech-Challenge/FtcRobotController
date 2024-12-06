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
    public static double angP;
    public static double angI;
    public static double angD;
    public static double sP;
    public static double sI;
    public static double sD;




    @Override
    public void init() {
        slide = new nematocyst(this);
        slide.init("pivot", "slide", "wrist", "claw");
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        slide.loop();
        slide.updatePID(angP, angI, angD);
        slide.getTelemetry();
        slide.getTelemetry(t2);
    }
}
