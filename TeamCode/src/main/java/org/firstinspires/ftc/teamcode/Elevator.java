package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator {

    private String ELEVATOR_MOTOR_NAME = "elevatorMotor";

    private double ELEVATOR_MOTOR_POWER = 2.0;

    private DcMotor elevatorMotor = null;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.dcMotor.get(ELEVATOR_MOTOR_NAME);
    }

    public void changeDirection(DcMotor.Direction direction) {
        elevatorMotor.setDirection(direction);
    }

    public void update(Gamepad gamepad) {
        double power = ((gamepad.dpad_up || gamepad.dpad_down) ? 1 : 0);
        if(gamepad.dpad_down){
            changeDirection(DcMotor.Direction.REVERSE);
        } else {
            changeDirection(DcMotor.Direction.FORWARD);
        }
        update(power);
    }

    public void update(double power) {
        elevatorMotor.setPower(ELEVATOR_MOTOR_POWER * power);
    }
}
