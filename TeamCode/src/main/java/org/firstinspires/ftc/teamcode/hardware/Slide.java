package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;



public class Slide {
    private DcMotorEx motor = null;
    private TouchSensor zeroTouch = null;
    private String motorName = "";
    private String zeroTouchName = "";
    private int maxTicks = 0;
    private double maxSpeed = 0.0;
    private double ticksPerInch = 0;
    private ExtendMotorDirection extendMotorDirection = ExtendMotorDirection.Forward;
    private boolean isHome = false;
    public enum ExtendMotorDirection {Forward, Reverse};
    public Slide(String motor, String zeroTouch, ExtendMotorDirection extendMotorDirection, int maxTicks, double maxSpeed, double ticksPerInch) {
        this.motorName = motor;
        this.zeroTouchName = zeroTouch;
        this.extendMotorDirection = extendMotorDirection;
        this.maxTicks = maxTicks;
        this.maxSpeed = maxSpeed;
        this.ticksPerInch = ticksPerInch;
    }

    public void Init(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, motorName);
        if(zeroTouchName.length() > 0) {
            zeroTouch = hardwareMap.get(TouchSensor.class, zeroTouchName);
        }
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(extendMotorDirection == ExtendMotorDirection.Forward ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE);
    }

    public void Extend(double power) {
        //extend with the power
        motor.setTargetPosition(maxTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.min(power, maxSpeed));
    }

    public void Retract(double power) {
        //extend with the power
        motor.setTargetPosition(-200);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.min(power, maxSpeed));
    }

    public double GetExtendedInches() {
        return motor.getCurrentPosition()*ticksPerInch;
    }

    public void MoveTo(double inches, double power) {
        //move to to provided inches
        int targetTicks = (int) (inches*ticksPerInch);
        targetTicks = Math.max(0, targetTicks);
        targetTicks = Math.min(targetTicks, maxTicks);
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Math.min(power, maxSpeed));
    }

    public void ProcessLoop() {
        if(!isHome && zeroTouch != null && zeroTouch.isPressed()) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            isHome = true;
        }else if(isHome && zeroTouch != null && !zeroTouch.isPressed()) {
            isHome = false;
        }
    }
    public  void Stop() {
        //stop the motor
        motor.setTargetPosition(0);
        motor.setPower(0);
    }
    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}

