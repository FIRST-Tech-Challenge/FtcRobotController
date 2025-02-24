package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.IDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@Config
public class DriveDistanceDriveCommand extends CommandBase {
    public static double Kp = 2;
    public static double Ki = 1.5;
    public static double Kd = 0.25;
    public static double Kf = 0.25;
    public static double MAX_POWER = 0.75;

    private final PIDFController pController;
    private final double finalPos;

    private double STARTING_POS;

    private final IDriveTrainSubsystem subsystem;

    public DriveDistanceDriveCommand(IDriveTrainSubsystem subsystem, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.finalPos = meters;

        this.pController = new PIDFController(Kp, Ki, Kd, Kf);
        this.pController.setTolerance(0.02);
    }

    @Override
    public void initialize() {
        super.initialize();
        this.subsystem.getDataLogger().addData(DataLogger.DataType.INFO, "DriveDistanceCommand: " + "Moving " + this.finalPos + " meters");
        this.STARTING_POS = this.subsystem.getForwardDistanceDriven();
        this.pController.reset();
        this.pController.setSetPoint(this.finalPos);
    }

    @Override
    public void execute() {
        super.execute();

        double driveDistance = (this.subsystem.getForwardDistanceDriven() - this.STARTING_POS);

        double rawPower = this.pController.calculate(driveDistance);
        rawPower += Math.signum(rawPower) * DriveConstants.Ks;

        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);

        MultipleTelemetry telemetry = this.subsystem.getTelemetry();
        telemetry.addData("----", this.getClass().getSimpleName() + " :----");
        telemetry.addData("Current Motor Power", power);
        telemetry.addData("Distance Driven", driveDistance);
        telemetry.addData("rel motor encoder", this.subsystem.getForwardDistanceDriven() - this.STARTING_POS);
        telemetry.addData("motor encoder", this.subsystem.getForwardDistanceDriven());
        telemetry.update();

        this.subsystem.moveSideMotors(-power, -power);
    }

    @Override
    public void end(boolean interrupted) {
        this.subsystem.moveSideMotors(0, 0);
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return this.pController.atSetPoint();
    }
}
