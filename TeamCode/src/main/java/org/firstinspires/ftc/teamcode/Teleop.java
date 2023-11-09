package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@TeleOp(name="Normal")
public class Teleop extends OpMode {

    TelemetryPacket packet = new TelemetryPacket();
    Drivetrain mecanum = new Drivetrain();
    @Override
    public void init() {
        mecanum.initMotors();
    }

    public void loop() {
        mecanum.drive();
    }

}