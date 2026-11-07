// File for exposing robot hardware methods
package org.firstinspires.ftc.teamcode.core.hardware;

// Imports
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.motors.MotorGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.enums.Motors;

import java.util.EnumMap;
import java.util.List;
import java.util.Map;


// Class for centralising and exposing hardware methods
public class Hardware {
    // Private variable for storing the hardware map
    private final HardwareMap hardwareMap;
    // Private variable to store a map of motors
    private final Map<Motors, Motor> motors = new EnumMap<>(Motors.class);
    // Private variable of all the hubs
    private final List<LynxModule> allHubs;


    // Constructor for setting up the hardware map
    public Hardware(HardwareMap hardwareMap) {
        if (hardwareMap == null) { throw new IllegalArgumentException("Hardware map cannot be null"); }
        this.hardwareMap = hardwareMap;
        this.allHubs = hardwareMap.getAll(LynxModule.class);

        this.init();
    }
    // Method for initialising hardware
    private void init() {
        for (LynxModule hub : this.allHubs) { hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL); }

        for (Motors motorEnum : Motors.values()) {
            MotorEx motor = new MotorEx(this.hardwareMap, motorEnum.getHardwareName());

            motor.setZeroPowerBehavior(motorEnum.getZeroPowerBehavior());
            motor.setCachingTolerance(motorEnum.getCachingTolerance());
            motor.setInverted(motorEnum.isInverted());

            this.motors.put(motorEnum, motor);
        }

        if (this.motors.containsKey(Motors.FLYWHEEL_1) && this.motors.containsKey(Motors.FLYWHEEL_2)) {
            MotorGroup flywheel = new MotorGroup(this.motors.get(Motors.FLYWHEEL_1), this.motors.get(Motors.FLYWHEEL_2));

            flywheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            flywheel.setRunMode(Motor.RunMode.VelocityControl);

            this.motors.put(Motors.FLYWHEEL_1, flywheel);
            this.motors.put(Motors.FLYWHEEL_2, flywheel);
        } else {
            throw new IllegalArgumentException("Flywheel motors are not initialized");
        }
    }
    // Method for inverting a motor's direction
    public void setMotorInverted(Motors motorEnum) {
        checkMotorExists(motorEnum);

        if (motorEnum != Motors.FRONT_LEFT && motorEnum != Motors.BACK_LEFT && motorEnum != Motors.FRONT_RIGHT && motorEnum != Motors.BACK_RIGHT) {
            boolean currentInverted = motorEnum.isInverted();

            this.motors.get(motorEnum).setInverted(!currentInverted);
        } else {
            throw new IllegalArgumentException("Motor " + motorEnum.getHardwareName() + " cannot be inverted");
        }
    }
    // Method for setting the run mode of a motor
    public void setMotorRunMode(Motors motorEnum, Motor.RunMode runMode) {
        checkMotorExists(motorEnum);

        if (motorEnum != Motors.FLYWHEEL_1 && motorEnum != Motors.FLYWHEEL_2) {
            this.motors.get(motorEnum).setRunMode(runMode);
        } else {
            throw new IllegalArgumentException("Motor " + motorEnum.getHardwareName() + " cannot have its run mode set");
        }
    }
    // Method for setting a motor's power
    public void setMotorPower(Motors motorEnum, double power) {
        checkMotorExists(motorEnum);

        if (power < -1.0 || power > 1.0) {
            throw new IllegalArgumentException("Power must be between -1 and 1");
        } else {
            this.motors.get(motorEnum).set(power);
        }
    }
    // Method for setting a motor's velocity
    public void setMotorVelocity(Motors motorEnum, double velocity) {
        checkMotorExists(motorEnum);

        if (this.motors.get(motorEnum) instanceof MotorEx) {
            ((MotorEx) this.motors.get(motorEnum)).setVelocity(velocity);
        } else {
            this.motors.get(motorEnum).set(velocity);
        }
    }
    // Method for setting a motor's velocity
    public void setMotorVelocity(Motors motorEnum, double velocity, AngleUnit angleUnit) {
        checkMotorExists(motorEnum);

        if (this.motors.get(motorEnum) instanceof MotorEx) {
            ((MotorEx) this.motors.get(motorEnum)).setVelocity(velocity, angleUnit);
        } else {
            this.motors.get(motorEnum).set(velocity);
        }
    }
    // Method for getting a motor's acceleration
    public double getMotorAcceleration(Motors motorEnum) {
        checkMotorExists(motorEnum);

        if (this.motors.get(motorEnum) instanceof MotorEx) { return ((MotorEx) this.motors.get(motorEnum)).getAcceleration(); }
        return 0.0;
    }
    // Method for getting a motor's velocity
    public double getMotorVelocity(Motors motorEnum) {
        checkMotorExists(motorEnum);

        if (this.motors.get(motorEnum) instanceof MotorEx) { return ((MotorEx) this.motors.get(motorEnum)).getVelocity(); }
        return 0.0;
    }
    // Method for getting a motor's corrected velocity
    public double getMotorCorrectedVelocity(Motors motorEnum) {
        checkMotorExists(motorEnum);

        if (this.motors.get(motorEnum) instanceof MotorEx) { return ((MotorEx) this.motors.get(motorEnum)).getCorrectedVelocity(); }
        return 0.0;
    }
    // Method for getting a motor's counts per revolution
    public double getMotorCPR(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getCPR();
    }
    // Method for getting a motor's current position
    public double getMotorCurrentPosition(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getCurrentPosition();
    }
    // Method for getting a motor's distance
    public double getMotorDistance(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getDistance();
    }
    // Method for getting whether a motor is inverted
    public boolean getMotorInverted(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getInverted();
    }
    // Method for getting a motor's max RPM
    public double getMotorMaxRPM(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getMaxRPM();
    }
    // Method for getting a motor's current raw power
    public double getMotorRawPower(Motors motorEnum) {
        checkMotorExists(motorEnum);

        return this.motors.get(motorEnum).getRawPower();
    }
    // Method for resetting Lynx cache
    public void resetLynxCache() { for (LynxModule hub : this.allHubs) { hub.clearBulkCache(); } }
    // Private method for checking if a motor exists
    private void checkMotorExists(Motors motorEnum) {
        if (this.motors.get(motorEnum) == null) { throw new IllegalArgumentException("Motor " + motorEnum.getHardwareName() + " is not initialized"); }
    }
}

// This file was created by: dev-verexxed
// Timestamp: [07/11/2026] [17:17]
