package com.bravenatorsrobotics.common.core;

import com.bravenatorsrobotics.common.drive.AbstractDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;

public class Robot<DriveType extends AbstractDrive> {

    public final LinearOpMode opMode;

    public final DriveType drive; // Drive System
    public final RobotSpecifications specifications; // Specifications

    protected final DcMotorEx[] driveMotors; // All the selected motors

    protected ArrayList<DcMotorEx> externalMotors; // Any motors that are not part of the drive system

    // Constructor
    public Robot(LinearOpMode opMode, RobotSpecifications specifications) {
        this.opMode = opMode;
        this.specifications = specifications;

        this.driveMotors = new DcMotorEx[specifications.robotMotors.length];
        this.externalMotors = new ArrayList<>();

        // Get the motors from the hardware map with correct direction
        for(int i = 0; i < specifications.robotMotors.length; i++) {
            boolean shouldReverse = false;

            if(specifications.robotMotors[i].startsWith("!")) {
                shouldReverse = true;
                specifications.robotMotors[i] = specifications.robotMotors[i].substring(1);
            }

            this.driveMotors[i] = opMode.hardwareMap.get(DcMotorEx.class, specifications.robotMotors[i]);
            if(shouldReverse)
                this.driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Create Instance of Drive
        try {
            Constructor<DriveType> constructor = ((Class<DriveType>) specifications.driveType).getConstructor(Robot.class);
            this.drive = constructor.newInstance(this);
        } catch (NoSuchMethodException | IllegalAccessException | InstantiationException | InvocationTargetException e) {
            throw new BravenatorRuntimeException(Arrays.toString(e.getStackTrace()));
        }

        // Motor Count Check
        if(specifications.robotMotors.length != drive.GetExpectedMotorCount()) {
            throw new BravenatorRuntimeException("Expected \"" + drive.GetExpectedMotorCount() +
                    "\", but \"" + specifications.robotMotors.length + "\" were specified.");
        }

        // Set Zero Power Behavior
        SetZeroBehavior(specifications.zeroPowerBehavior);
    }

    // ==========================================================================================
    // Control Methods
    // ==========================================================================================

    public void SetRunMode(DcMotorEx.RunMode runMode) {
        for(DcMotorEx motor : driveMotors)
            motor.setMode(runMode);
    }

    public void SetZeroBehavior(DcMotor.ZeroPowerBehavior behavior) {
        for(DcMotorEx motor : driveMotors)
            motor.setZeroPowerBehavior(behavior);
    }

    public void Reset() {
        SetRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotorEx GetMotor(String motorName, boolean isReversed) {
        DcMotorEx motor = opMode.hardwareMap.get(DcMotorEx.class, motorName);
        motor.setDirection(isReversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        externalMotors.add(motor);

        return motor;
    }

    public void Stop() {
        for(DcMotorEx motor : driveMotors) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        for(DcMotorEx externalMotor : externalMotors) {
            externalMotor.setPower(0);
            externalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    // ==========================================================================================
    // Getters and Setters
    // ==========================================================================================

    public DcMotorEx[] GetDriveMotors() { return driveMotors; }
}