package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake1 {
    private Color colorSensor;
    private DcMotor motor;
    private Telemetry telemetry;

    public Intake1(HardwareMap hardwareMap, Telemetry telemetryGet) {
        colorSensor = new Color(hardwareMap, telemetry);
        motor = hardwareMap.get(DcMotor.class, "leftMotor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = telemetryGet;
    }

    public void waitForGamePiece() {
        while (colorSensor.colorSenseRed() < 400) {
            motor.setPower(-0.5);
        }
        motor.setPower(0);


    }
}