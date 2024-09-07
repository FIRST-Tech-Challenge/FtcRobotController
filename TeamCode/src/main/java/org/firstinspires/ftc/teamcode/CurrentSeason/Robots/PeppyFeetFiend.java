package org.firstinspires.ftc.teamcode.CurrentSeason.Robots;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.roboctopi.cuttlefish.controller.MecanumController;

import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Intake;
import org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem.Outtake;

public class PeppyFeetFiend extends AbstractRobot {
    public Intake intake;
    public Outtake outtake;


    public PeppyFeetFiend(OpMode opMode) {
        super(opMode);

        outtake = new Outtake(this);
        intake = new Intake(this);
    }
}