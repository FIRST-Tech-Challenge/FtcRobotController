package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    //public final Intake intake;
    public final Outtake outtake;
    public SmartGamepad smartGamepad1;
    public SmartGamepad smartGamepad2;

    public CrabRobot(LinearOpMode opMode) {
        super(opMode);
        //mecanumDrive = new SimpleMecanumDrive(this);
        mecanumDrive = new SimpleMecanumDrive(this);
        registerSubsystem(mecanumDrive);
        //intake = new Intake(this);
        //registerSubsystem(intake);
        outtake = new Outtake(this);
        registerSubsystem(outtake);

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