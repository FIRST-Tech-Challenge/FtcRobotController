package org.firstinspires.ftc.teamcode.subsystems.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MecanumDriveParameters {
    public int [] FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
    public int [] ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
    public int [] REVERSED_WHEELS; // reversed motors.
    public DcMotor[] motors;
    public Telemetry telemetry;
}
