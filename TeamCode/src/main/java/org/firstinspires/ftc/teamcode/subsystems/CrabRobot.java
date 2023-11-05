package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CrabRobot extends Robot {
    public final Intake intake;
    public CrabRobot(LinearOpMode opMode) {
        super(opMode);
        //mecanumDrive = new SimpleMecanumDrive(this);
        intake = new Intake(this);
        registerSubsystem(intake);


    }
}