package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class RobotPart {

    protected Telemetry.Item telemetry;

    protected Map<String, DcMotorEx> motors;
    protected Map<String, Servo> servos;
    protected Map<String, CRServo> crServos;

    public void resetEncoders() {
        for (DcMotorEx motor : motors.values()) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void setPower(int power) {
        for (DcMotorEx motor : motors.values()) {
            motor.setPower(power);
        }
    }

}
