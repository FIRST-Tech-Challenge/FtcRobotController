package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.BaseRobot;

public class LinearActuator {
    public final DcMotor actuatorMotor;
    public final Servo actuatorServo;
    private final HardwareMap hardwareMap;
    private final double speed = 1;
    private final double maxActuatorLength = 14500;
    private final double uprightValue = 0.45;
    private final double restingValue = 1;
    private final BaseRobot baseRobot;

    public LinearActuator(BaseRobot baseRobot) {
        this.baseRobot = baseRobot;
        this.hardwareMap = baseRobot.hardwareMap;
        actuatorMotor = hardwareMap.get(DcMotor.class, "linearActuator");
        actuatorMotor.setDirection(DcMotor.Direction.REVERSE);
        actuatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // Reset the motor encoder
        actuatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // Turn the motor back on when we are done
        actuatorServo = hardwareMap.get(Servo.class, "actuatorServo");
        actuatorServo.setPosition(restingValue);
    }

    public void extend() {
        if (!((actuatorServo.getPosition() < uprightValue + 0.05) && (actuatorServo.getPosition() > uprightValue - 0.05))) {
            return;
        }
        if (actuatorMotor.getCurrentPosition() < maxActuatorLength - 100) {
            actuatorMotor.setPower(speed);
        } else {
            actuatorMotor.setPower(0);
        }
    }

    public void retract() {
        if (actuatorMotor.getCurrentPosition() > 50) {
            actuatorMotor.setPower(-speed);
        } else {
            actuatorMotor.setPower(0);
        }
    }

    public void stop() {
        actuatorMotor.setPower(0);
    }

    public void changePosition() {
        if (actuatorMotor.getCurrentPosition() > 100) {
            return;
        }
        if (actuatorServo.getPosition() == restingValue) {
            actuatorServo.setPosition(uprightValue);
        } else {
            actuatorServo.setPosition(restingValue);
        }
    }


}
