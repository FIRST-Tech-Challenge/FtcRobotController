package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSub;

/**
 * This command is dedicated to driving forwards/backwards a certain amount of distance and speed
 */

public class DriveDistanceCmd extends CommandBase {

    private final DrivetrainSub drivetrainSub;
    private final double driveDistance;
    private final double driveSpeed;
    private Telemetry telemetry;

    /**
     * Drives the robot forward/backwards for a certain distance and speed.
     *
     * @param driveDistanceParam The number of inches the robot will drive
     * @param driveSpeedParam  The speed at which the robot will drive
     * @param drivetrainSubParam  The drive subsystem on which this command will run
     * @param telemetryParam Telemetry input
     */

    public DriveDistanceCmd(double driveDistanceParam, double driveSpeedParam, DrivetrainSub drivetrainSubParam, Telemetry telemetryParam) {
        driveDistance = driveDistanceParam;
        driveSpeed = driveSpeedParam;
        this.drivetrainSub = drivetrainSubParam;
        this.telemetry = telemetryParam;
        addRequirements(drivetrainSubParam);
    }

    @Override
    public void initialize() {
        telemetry.addLine("Drive Distance Initialized");
        drivetrainSub.resetEncoders();
        if (driveDistance > 0){
            drivetrainSub.move(driveSpeed, 0);
        }else{
            drivetrainSub.move(-driveSpeed, 0);
        }

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("End Called");
        drivetrainSub.getDrive().stop();
        //drivetrainSub.move(0, 0);
    }


    @Override
    public boolean isFinished() {
        System.out.println("Encoder Distance: " + drivetrainSub.getFrontLeftEncoderDistance());
        System.out.println("Is done? "+ (Math.abs(drivetrainSub.getFrontLeftEncoderDistance()) >= Math.abs(driveDistance)));
        return Math.abs(drivetrainSub.getFrontLeftEncoderDistance()) >= Math.abs(driveDistance);
    }

}
