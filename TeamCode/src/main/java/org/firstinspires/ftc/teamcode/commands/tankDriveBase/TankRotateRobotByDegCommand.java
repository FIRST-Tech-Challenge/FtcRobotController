package org.firstinspires.ftc.teamcode.commands.tankDriveBase;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PController;

import org.firstinspires.ftc.teamcode.subsystems.TankDriveBaseSubSystem;

public class TankRotateRobotByDegCommand extends CommandBase {
    private TankDriveBaseSubSystem driveBaseSubsystem;
    private PController pController;

    public static final double defaultKp = 0.05f;
    private static final double MAX_POWER = 0.3f;

    private int timesDone = 0;

    private final double degToRotate;


    private double STARTING_ANGLE;

    public TankRotateRobotByDegCommand(TankDriveBaseSubSystem driveBaseSubsystem, double degToRotate, double kp) {
        addRequirements(driveBaseSubsystem);
        this.driveBaseSubsystem = driveBaseSubsystem;
        this.pController = new PController(kp);
        this.pController.setTolerance(2.5);
        this.degToRotate = degToRotate;
    }

    public TankRotateRobotByDegCommand(TankDriveBaseSubSystem driveBaseSubsystem, double degToRotate) {
        this(driveBaseSubsystem, degToRotate, defaultKp);
    }

    @Override
    public void initialize() {
        this.STARTING_ANGLE = this.driveBaseSubsystem.getHeadingByGyro();
        this.pController.setSetPoint(Math.IEEEremainder(degToRotate + STARTING_ANGLE, 360));
    }

    @Override
    public void execute() {
        double headingDist = this.driveBaseSubsystem.getHeadingByGyro();
        double distLeft = Math.IEEEremainder(this.pController.getSetPoint() - headingDist, 360);


        double rawPower = this.pController.calculate(this.pController.getSetPoint() - distLeft);


        double power = Math.min(Math.max(rawPower, -MAX_POWER), MAX_POWER);


        FtcDashboard.getInstance().getTelemetry().addData("power", power);
        FtcDashboard.getInstance().getTelemetry().addData("dist left", distLeft);
        FtcDashboard.getInstance().getTelemetry().addData("heading dist", headingDist);
        FtcDashboard.getInstance().getTelemetry().addData("rel heading", this.driveBaseSubsystem.getHeadingByGyro() - this.STARTING_ANGLE);
        FtcDashboard.getInstance().getTelemetry().addData("heading", this.driveBaseSubsystem.getHeadingByGyro());
        FtcDashboard.getInstance().getTelemetry().update();

        this.driveBaseSubsystem.moveMotors(-power, power);
    }

    @Override
    public void end(boolean interrupted) {
        this.driveBaseSubsystem.moveMotors(0, 0);
    }

    @Override
    public boolean isFinished() {
        if (this.pController.atSetPoint()) {
            this.timesDone++;
        } else {
            this.timesDone = 0;
        }
        return this.timesDone > 5;
    }
}
