package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.AutoOpModeBase;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;


public class AutonomousCommand extends CommandBase {

    private AutoOpModeBase opMode;
    private AutoMecanumDriveTrain driveTrain;
    private RollingIntake intake;
    private LimeLight vision;
    private Telemetry telemetry;

    private DeliveryPivot pivot;

    private DeliverySlider slider;

    public AutonomousCommand(AutoOpModeBase opMode, AutoMecanumDriveTrain driveTrain, LimeLight vision, RollingIntake intake, DeliveryPivot pivot, DeliverySlider slider, Telemetry telemetry) {
        this.opMode = opMode;
        this.driveTrain = driveTrain;
        this.vision = vision;
        this.intake = intake;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;

        addRequirements(driveTrain, vision, intake, pivot, slider);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        try {
            opMode.executeOpMode();
        } catch (Exception e) {
        }
        finally {
            pivot.MoveToStartInAuto();
        }
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}