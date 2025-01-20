package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class CreepIntake extends CommandBase {
    private Intake intake;
    private double desiredPos;
    private double speed;
    private Timing.Timer timer;

    private double lastPos;
    private Telemetry telemetry;

    public CreepIntake(Intake intake, double desiredPos, double time2Max, Telemetry telemetry){
        this.intake = intake;
        this.desiredPos = desiredPos;
        this.speed = (double) 1 / time2Max;
        this.telemetry = telemetry;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        // Get current servo position
        double servoPos = intake.getExtensionPosition();
        lastPos = servoPos;
        // calculate difference between desired pos and current pos
        double differPos = desiredPos - servoPos;
        // calculate how long it will take to get there based on set speed
        double timeToGoal = (differPos/speed) * 1000;
        // double timeToGoal = ...

        this.timer = new Timing.Timer((long)timeToGoal, TimeUnit.MILLISECONDS);
        this.timer.start();
    }

    @Override
    public void execute() {
        long elapsedMillis = this.timer.elapsedTime();


        double pos = lastPos + (speed / (double) 1000) * elapsedMillis;
        if (pos >= this.desiredPos) {
            pos = this.desiredPos;
        }
        this.telemetry.addData("Extension: ", pos);
        this.intake.setExt(pos);
        lastPos = pos;
    }

    @Override
    public boolean isFinished() {
        return this.timer.done();
    }
}

