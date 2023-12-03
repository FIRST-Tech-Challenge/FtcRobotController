package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    public final Intake intake;
    public final Outtake outtake;
    //public final DualMotorLift dualMotorLift;
    public SmartGamepad smartGamepad1;
    public SmartGamepad smartGamepad2;
    public DroneLauncher droneLauncher;

    public CrabRobot(LinearOpMode opMode) {
        super(opMode);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        intake = new Intake(this);
        registerSubsystem(intake);
        outtake = new Outtake(this, opMode.telemetry);
        registerSubsystem(outtake);
        droneLauncher = new DroneLauncher(this, opMode.telemetry);
        registerSubsystem(droneLauncher);
        //dualMotorLift = new DualMotorLift(this, opMode.telemetry, DualMotorLift.Mode.BOTH_MOTORS_PID);
        //registerSubsystem(dualMotorLift);

    }

    public void addGamepads(Gamepad g1, Gamepad g2){
        smartGamepad1 = new SmartGamepad(g1);
        if (smartGamepad1 != null ) {
            //Log.v("update", "registering smartGamepad1: " + smartGamepad1.getClass().getSimpleName());
            registerSubsystem(smartGamepad1);
        }
        smartGamepad2 = new SmartGamepad(g2);
        if (smartGamepad2 != null) {
            //Log.v("update", "registering smartGamepad2: " + smartGamepad2.getClass().getSimpleName());
            registerSubsystem(smartGamepad2);
        }
    }
}

/*
CONFIG

CONTROL HUB
    Motors
    0 - DriveRF
    1 - DriveLF
    2 - DriveLR
    3 - DriveRR

    Servos
    0 - droneTrigger
    1
    2 - intakeServoR
    3 - intakeServoL
    4


EXPANSION HUB
    Motors
    0 - intakeMotor
    1
    2 - slideLt
    3 - slideRt

    Servos
    0 - armServo_Left
    1 - armServo_Right
    2 - dumpServo
    3
    4


 */