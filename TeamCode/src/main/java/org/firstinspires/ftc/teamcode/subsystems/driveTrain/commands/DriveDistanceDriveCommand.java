package org.firstinspires.ftc.teamcode.subsystems.driveTrain.commands;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.driveTrain.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain.IDriveTrainSubsystem;
import org.firstinspires.ftc.teamcode.util.DataLogger;

@Config
public class DriveDistanceDriveCommand extends CommandBase {
    public static final double Kp = 0.65;
    private static final double MAX_POWER = 0.8;

    private final PController pController;
    private final double finalPos;

    private double STARTING_POS;

    private final IDriveTrainSubsystem subsystem;

    public DriveDistanceDriveCommand(IDriveTrainSubsystem subsystem, double meters) {
        this.subsystem = subsystem;
        addRequirements(subsystem);

        this.finalPos = meters;

        this.pController = new PController(Kp);
        this.pController.setTolerance(0.05);
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
        telemetry.addData("power", power);
        telemetry.addData("dist", driveDistance);
        telemetry.addData("rel motor encoder", this.subsystem.getForwardDistanceDriven() - this.STARTING_POS);
        telemetry.addData("motor encoder", this.subsystem.getForwardDistanceDriven());
        telemetry.update();

        this.subsystem.moveSideMotors(power, power);
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
