package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;


public class IntakeClaw{
    Telemetry telemetry;
    public Servo IntakeMotor;

    public IntakeClaw(HardwareMap hardwareMap, String IntakeMotorName, Telemetry telemetry) {
        this.telemetry = telemetry;
        setup(hardwareMap, IntakeMotorName);
    }

    public void setup(HardwareMap hardwareMap, String IntakeMotorName) {
        IntakeMotor = hardwareMap.get(Servo.class, IntakeMotorName);

    }

    public void setClawPosition(double position) {
        IntakeMotor.setPosition(position);
    }
}