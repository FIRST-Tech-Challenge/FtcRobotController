package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Arm  extends SubsystemBase {
    public Motor motor;
    private final Telemetry telemetry;

    public Arm(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = new Motor(hardwareMap, "hdHexMotor");
        resetEncoders();
    }
    public void resetEncoders() {
        motor.setRunMode(Motor.RunMode.RawPower);
    }

    @Override
    public void periodic() {
        telemetry.addData("Arm Position", getPosition());
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public int getPosition() {
        return motor.getCurrentPosition();
    }

    public  void stopMotor() {
        motor.stopMotor();
    }
}
