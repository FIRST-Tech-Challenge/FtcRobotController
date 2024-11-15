package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.voltageToAngleConstants;

import java.io.IOException;

@TeleOp
public class test extends OpMode {
    voltageToAngleConstants angles;
    FtcDashboard dashboard;
    Telemetry telemetry2;
    String[] encoders = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    @Override
    public void init() {
//        try {
        angles = new voltageToAngleConstants(this, hardwareMap, encoders);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
        dashboard = FtcDashboard.getInstance();
        telemetry2 = dashboard.getTelemetry();
    }
    public void init_loop() {
        angles.init_loop();
    }
    @Override
    public void loop() {
        angles.loop();
        angles.getTelemetry(telemetry2);
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

    }
}
