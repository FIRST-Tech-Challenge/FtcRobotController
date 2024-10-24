package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotorEx motor = null;
    private String motorName = "";
    private int maxTicks = 0;
    private double maxSpeed = 0.0;
    private double ticksPerInch = 0;

    Slide(String motor, int maxTicks, double maxSpeed, double ticksPerInch) {
        this.motorName = motor;
        this.maxTicks = maxTicks;
        this.maxSpeed = maxSpeed;
        this.ticksPerInch = ticksPerInch;
    }

    public void Init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void Extend(double power) {
        //extend with the power
        motor.setTargetPosition(maxTicks);
        motor.setPower(Math.min(power, maxSpeed));
    }

    public void Retract(double power) {
        //extend with the power
        motor.setTargetPosition(0);
        motor.setPower(Math.min(power, maxSpeed));
    }

    public double GetExtendedInches() {
        return motor.getCurrentPosition()*ticksPerInch;
    }

    public void MoveTo(double inches, double power) {
        //move to to provided inches
    }
    public  void Stop() {
        //stop the motor
        motor.setTargetPosition(0);
        motor.setPower(0);
    }
}

