package org.firstinspires.ftc.teamcode.Elevator;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ElevatorControl {
    public DcMotor elevatorMotor;
    private Telemetry telemetry;

    public ElevatorControl(HardwareMap hardwareMap, Telemetry telemetryGet) {
        elevatorMotor = hardwareMap.get(DcMotor.class, "leftBack");

        telemetry = telemetryGet;
    }

    public void controlElevator(Double power) {
        elevatorMotor.setPower(power);
    }
}