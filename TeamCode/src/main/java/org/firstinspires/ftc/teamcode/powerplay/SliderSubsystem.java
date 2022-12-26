package org.firstinspires.ftc.teamcode.powerplay;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SliderSubsystem extends SubsystemBase {
    private final MotorEx motor;

    public enum Level {
        PARK, INTAKE, ONE, TWO, THREE;
    }

    private Level level;

    private int[] levelPositions = {0, 200, 533, 1121, 1678};

    public SliderSubsystem(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "slider");
        motor.setInverted(true);
        motor.setRunMode(MotorEx.RunMode.PositionControl);
        motor.setZeroPowerBehavior(MotorEx.ZeroPowerBehavior.BRAKE);
        motor.setPositionTolerance(10);   // allowed maximum error
        level = Level.PARK;
        setLevel(Level.PARK);
    }

    public void run() {
        motor.set(0.1);
    }

    public void stop() {
        motor.set(0);
    }

    public void setLevel(Level levelPicked) {
        level = levelPicked;
        int levelIdx = 1;

        if(level == Level.ONE)
            levelIdx = 2;
        else if (level == Level.TWO)
            levelIdx = 3;
        else if (level == Level.THREE)
            levelIdx = 4;
        else if (level == Level.INTAKE)
            levelIdx = 0;

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

    public void setHeight(int pos) {
        motor.setTargetPosition(pos);
    }

    public Level getLevel() {
        return level;
    }

    public boolean atTargetLevel() {
        return motor.atTargetPosition();
    }
}
