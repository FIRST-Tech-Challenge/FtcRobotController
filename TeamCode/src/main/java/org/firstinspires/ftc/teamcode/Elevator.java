package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator {

    private String ELEVATOR_MOTOR_NAME = "elevatorMotor";

    private double ELEVATOR_MOTOR_POWER = 0.5;

    private DcMotor elevatorMotor = null;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor = hardwareMap.dcMotor.get(ELEVATOR_MOTOR_NAME);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void update(Gamepad gamepad) {
        double power = 0;
        if(gamepad.dpad_up){
            ++power;
        } else if (gamepad.dpad_down){
            --power;
        }
        update(power);
    }

    public void update(double power) {
        elevatorMotor.setPower(ELEVATOR_MOTOR_POWER * power);
    }
}
