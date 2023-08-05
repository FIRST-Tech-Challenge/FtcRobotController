package org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class ElevatorSubsystem extends SubsystemBase {
    private final MotorEx motor;
    public static double POS_KP = 0.16;

    private double KS = 0, KV = 0, KA = 0;

    public enum Level {
        LOW, TRAVEL, MID, HIGH, AUTO_SCORING;
    }

    private Level level;

    private int[] levelPositions = {0, 120, 620, 2000, 2220};

    public ElevatorSubsystem(HardwareMap hm) {
        motor = new MotorEx(hm, "slider");
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(10);   // allowed maximum error
        motor.setPositionCoefficient(POS_KP);

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
    public void setAuto() {
        motor.setRunMode(Motor.RunMode.PositionControl);
        motor.setTargetPosition(motor.getCurrentPosition());
    }

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
