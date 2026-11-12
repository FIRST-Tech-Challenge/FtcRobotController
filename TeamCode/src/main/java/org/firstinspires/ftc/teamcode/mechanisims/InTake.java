package org.firstinspires.ftc.teamcode.mechanisims;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HardwareNames;

public class InTake {
    DcMotorEx motor;
    private final static double TARGET_VELOCITY = 1125;

    public InTake(HardwareMap hardwareMap) {

        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.INTAKE_MOTOR);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void update(double speedMultiplier){
        motor.setVelocity(speedMultiplier * TARGET_VELOCITY);
    }
}
