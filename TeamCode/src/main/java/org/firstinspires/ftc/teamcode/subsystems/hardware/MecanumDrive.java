package org.firstinspires.ftc.teamcode.subsystems.hardware;

import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public interface MecanumDrive extends SubSystem {
    void moveRect(double forward, double lateral, double rotate);
    void movePolar(double power, double angle, double rotate);
    void outputTelemetry(MecanumDriveTelemetryTypes type);
}
