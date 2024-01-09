package org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.hardware.motors.MotorGroup;
import com.arcrobotics.ftclib.util.MathUtils;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.inventors.ftc.robotbase.hardware.MotorExEx;

public class ElevatorSubsystem extends SubsystemBase {
    private final MotorExEx leftMotor;
    private final MotorExEx rightMotor;
    private final MotorGroup motors;

    private boolean isAutoEnabled = true;

    private PIDFController controller;
    private double calculation = 0.0;

    public double MAX_SPEED = 0.5; // TODO: Speed Value Might Change

    public enum Level {
        LOADING, LOW, MID, HIGH;
    }

    private Level level;

    private int[] levelPositions = {0, 120, 620, 2000}; // TODO: Level Values Might Change

    public ElevatorSubsystem(HardwareMap hm) {
        leftMotor = new MotorExEx(hm, "slider_left", 383.6, 435);
        rightMotor = new MotorExEx(hm, "slider_right", 383.6, 435);
        leftMotor.setMaxPower(MAX_SPEED);
        rightMotor.setMaxPower(MAX_SPEED);
        leftMotor.setInverted(true);
        motors = new MotorGroup(leftMotor, rightMotor);

        motors.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motors.setPositionTolerance(15); // TODO: allowed maximum error value might change
        motors.setPositionCoefficient(0.16); // TODO: Kp Value might change

        motors.resetEncoder();

        setAuto();
        level = Level.LOADING;
        setLevel(Level.LOADING);
    }

//    @Override
//    public void periodic() {
//        calculation = MathUtils.clamp(controller.calculate(getHeight()), -MAX_SPEED, MAX_SPEED);
//        run();
//    }

    public void run() {
        motors.set(MAX_SPEED);
    }

    public void stop() {
        motors.stopMotor();
    }

    public void setLevel(Level levelPicked) {
        level = levelPicked;
        int levelIdx = 0;

        if(level == Level.LOADING)
            levelIdx = 0;
        else if (level == Level.LOW)
            levelIdx = 1;
        else if (level == Level.MID)
            levelIdx = 2;
        else if (level == Level.HIGH)
            levelIdx = 3;

        motors.setTargetPosition(levelPositions[levelIdx]);
    }

    public void setManual() {
        motors.setRunMode(Motor.RunMode.RawPower);
    }

    public void setAuto() {
        motors.setRunMode(Motor.RunMode.PositionControl);
    }

    public void setPower(double power) {
        motors.set(power);
    }

    public Double getHeight(){
        return motors.getPositions().get(0);
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTargetLevel() {
        return motors.atTargetPosition();
    }
}
