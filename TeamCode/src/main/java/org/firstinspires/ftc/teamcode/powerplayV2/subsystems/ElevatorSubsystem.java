package org.firstinspires.ftc.teamcode.powerplayV2.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ElevatorSubsystem extends SubsystemBase {
    private final MotorEx motor;

    private double KP = 0.08;

    private double KS = 0, KV = 0, KA = 0;

    public enum Level {
        LOW, TRAVEL, MID, HIGH, AUTO_SCORING;
    }

    private Level level;

    private int[] levelPositions = {0, 120, 620, 2000, 2220};

    public ElevatorSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "slider");
//        motor.setInverted(true);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(10);   // allowed maximum error
        motor.setPositionCoefficient(KP);

        // Velocity Control
        motor.setFeedforwardCoefficients(KS, KV, KA);

        motor.resetEncoder();

        setAuto();
        level = Level.LOW;
        setLevel(Level.LOW);
    }

    public void run() {
        motor.set(0.15);
    }

    public void stop() {
        motor.set(0);
    }

    public void setLevel(Level levelPicked) {
        level = levelPicked;
        int levelIdx = 0;

        if(level == Level.LOW)
            levelIdx = 0;
        else if (level == Level.TRAVEL)
            levelIdx = 1;
        else if (level == Level.MID)
            levelIdx = 2;
        else if (level == Level.HIGH)
            levelIdx = 3;
        else if (level == Level.AUTO_SCORING)
            levelIdx = 4;

        motor.setTargetPosition(levelPositions[levelIdx]);
    }

    public void setManual() { motor.setRunMode(Motor.RunMode.RawPower); }
    public void setAuto() { motor.setRunMode(Motor.RunMode.PositionControl); }

    public void setPower(double power) {
        motor.set(power);
    }

    public int getHeight(){
        return motor.getCurrentPosition();
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTargetLevel() {
        return motor.atTargetPosition();
    }
}
