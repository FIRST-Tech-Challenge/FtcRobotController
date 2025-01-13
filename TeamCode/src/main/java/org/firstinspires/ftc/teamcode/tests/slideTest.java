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
    public static double angP = 0.00425;
    public static double angI = 0;
    public static double angD = 8e-6;
    public static double angCos = 0.3375;
    public static double angExt = 1e-7;
    public static double sP = 0;
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
        slide.updatePID(angP, angI, angD, angCos, angExt);
        slide.updateSlidePID(sP, sI, sD);
        slide.getTelemetry();
        slide.getTelemetry(t2);
    }
}
