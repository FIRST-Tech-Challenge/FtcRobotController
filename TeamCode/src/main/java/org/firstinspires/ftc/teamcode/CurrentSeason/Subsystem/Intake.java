package org.firstinspires.ftc.teamcode.CurrentSeason.Subsystem;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractRobot;
import org.firstinspires.ftc.teamcode.AbstractClasses.AbstractSubsystem;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;
import org.firstinspires.ftc.teamcode.CurrentSeason.Robots.PeppyFeetFiend;
import org.firstinspires.ftc.teamcode.CurrentSeason.Util.Toggle;

import java.io.IOException;

public class Intake extends AbstractSubsystem {
    PeppyFeetFiend robot;
    public DcMotor intakeMotor;
    public Toggle intake_toggle = new Toggle(false);
    public Intake(AbstractRobot robot) {
        super(robot);
        this.robot = (PeppyFeetFiend) robot;

    }

    @Override
    public void init() {
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {

    }

    @Override
    public void driverLoop() {
        //use for telemetry
    }

    @Override
    public void stop() {

    }
}