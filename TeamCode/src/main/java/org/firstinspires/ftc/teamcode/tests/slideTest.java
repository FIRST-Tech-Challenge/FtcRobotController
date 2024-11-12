package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.linearSlide;

public class slideTest extends OpMode {
    linearSlide slide;
    FtcDashboard dash;
    Telemetry t2;
    @Override
    public void init() {
        slide = new linearSlide(this);
        slide.init();
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        slide.loop();
        slide.getTelemetry();
        slide.getTelemetry(t2);
    }
}
