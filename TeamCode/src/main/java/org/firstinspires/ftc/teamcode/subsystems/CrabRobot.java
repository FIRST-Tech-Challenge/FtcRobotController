package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final SimpleMecanumDrive mecanumDrive;
    //public final Intake intake;
    public final Outtake outtake;
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
}