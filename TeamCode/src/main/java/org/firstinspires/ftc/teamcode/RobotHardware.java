package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


public class RobotHardware{

    protected DcMotor left_motor;
    protected DcMotor right_motor;
    protected Servo test_servo;

    public void init(@NonNull HardwareMap hwMap) {
        left_motor = hwMap.dcMotor.get("left_motor");
        right_motor = hwMap.dcMotor.get("right_motor");
//        test_servo = hwMap.servo.get("test_servo");

        right_motor.setDirection(DcMotor.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
