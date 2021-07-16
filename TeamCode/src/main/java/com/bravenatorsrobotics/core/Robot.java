package com.bravenatorsrobotics.core;

import com.bravenatorsrobotics.drive.AbstractDrive;
import com.bravenatorsrobotics.drive.FourWheelDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;

public class Robot {

    public final AbstractDrive drive; // Drive System
    public final RobotSpecifications specifications; // Specifications

    protected final DcMotorEx[] motors; // All the selected motors

    // Constructor
    public Robot(HardwareMap hardwareMap, RobotSpecifications specifications) {
        this.specifications = specifications;

        this.motors = new DcMotorEx[specifications.robotMotors.length];

        // Get the motors from the hardware map with correct direction
        for(int i = 0; i < specifications.robotMotors.length; i++) {
            boolean shouldReverse = false;

            if(specifications.robotMotors[i].startsWith("!")) {
                shouldReverse = true;
                specifications.robotMotors[i] = specifications.robotMotors[i].substring(1);
            }

            this.motors[i] = hardwareMap.get(DcMotorEx.class, specifications.robotMotors[i]);
            if(shouldReverse)
                this.motors[i].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        AbstractDrive drive;

        try {
            Constructor<?> constructor = specifications.driveType.getConstructor(specifications.driveType);
            drive = (AbstractDrive) constructor.newInstance(this);
        } catch (NoSuchMethodException | IllegalAccessException | InstantiationException | InvocationTargetException e) {
            e.printStackTrace();
            drive = new FourWheelDrive(this);
        }

        this.drive = drive;

        // Reset All Encoders
        SetRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // ==========================================================================================
    // Control Methods
    // ==========================================================================================

    public void SetRunMode(DcMotorEx.RunMode runMode) {
        for(DcMotorEx motor : motors)
            motor.setMode(runMode);
    }

    public void SetZeroBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for(DcMotorEx motor : motors)
            motor.setZeroPowerBehavior(behavior);
    }

    // ==========================================================================================
    // Getters and Setters
    // ==========================================================================================

    public DcMotorEx[] GetAllMotors() { return motors; }
}