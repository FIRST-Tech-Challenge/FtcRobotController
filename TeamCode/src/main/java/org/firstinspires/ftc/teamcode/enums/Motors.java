// File to centralise motors
package org.firstinspires.ftc.teamcode.enums;

// Imports
import com.seattlesolvers.solverslib.hardware.motors.Motor;


// Enum for creating a motor type
public enum Motors {
    // Types of motors
    FRONT_LEFT("front_left", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.05, true),
    FRONT_RIGHT("front_right", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.05, false),
    BACK_LEFT("back_left", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.05, true),
    BACK_RIGHT("back_right", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.05, false),
    INTAKE("intake", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.1, false),
    TRANSFER("transfer", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.RawPower, 0.1, false),
    FLYWHEEL_1("flywheel_1", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.VelocityControl, 0.001, false),
    FLYWHEEL_2("flywheel_2", Motor.GoBILDA.BARE, Motor.ZeroPowerBehavior.BRAKE, Motor.RunMode.VelocityControl, 0.001, false);

    // Private variable to store the motor hardware
    private final String hardwareName;
    // Private variable to store the motor type
    private final Motor.GoBILDA motorType;
    // Private variable to store the zero power behaviour of the motor
    private final Motor.ZeroPowerBehavior zeroPowerBehavior;
    // Private variable to store the run mode of the motor
    private final Motor.RunMode runMode;
    // Private variable to store the caching tolerance of the motor
    private final double cachingTolerance;
    // Private variable to store whether the motor direction should be inverted
    private final boolean inverted;

    // Constructor to set up motor types
    Motors(String hardwareName, Motor.GoBILDA motorType, Motor.ZeroPowerBehavior zeroPowerBehavior, Motor.RunMode runMode, double cachingTolerance, boolean inverted) {
        this.hardwareName = hardwareName;
        this.motorType = motorType;
        this.zeroPowerBehavior = zeroPowerBehavior;
        this.runMode = runMode;
        this.cachingTolerance = cachingTolerance;
        this.inverted = inverted;
    }
    // Method for getting the name of a motor
    public String getHardwareName() { return this.hardwareName; }
    // Method for getting the type of the motor
    public Motor.GoBILDA getMotorType() {
        return this.motorType;
    }
    // Method for getting the zero power behaviour of a motor
    public Motor.ZeroPowerBehavior getZeroPowerBehavior() { return this.zeroPowerBehavior; }
    // Method for getting the run mode of a motor
    public Motor.RunMode getRunMode() { return this.runMode; }
    // Method for getting the caching tolerance of a motor
    public double getCachingTolerance() { return this.cachingTolerance; }
    // Method for getting the inverted status of a motor
    public boolean isInverted() { return this.inverted; }
}

// This file was created by: dev-verexxed
// Timestamp: [05/11/2026] [12:06]
