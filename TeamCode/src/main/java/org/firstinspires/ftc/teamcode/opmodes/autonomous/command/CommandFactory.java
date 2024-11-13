package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.google.common.util.concurrent.Uninterruptibles;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

import java.util.concurrent.TimeUnit;

public class CommandFactory {

    private static final PIDController xPIDController = new PIDController(0.015, 0, 0.05);
    private static final PIDController yPIDController = new PIDController(0.015, 0, 0.05);
    private static final ProfiledPIDController rotPIDController = new ProfiledPIDController(0.015, 0, .05, new TrapezoidProfile.Constraints(
            Math.PI * 2,
            Math.PI
    ));

    private static final double MAX_VELOCITY = 400;

    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, 1);



    private AutoMecanumDriveTrain driveTrain;
    private RollingIntake intake;
    private LimeLight vision;
    private Telemetry telemetry;
    private DeliveryPivot pivot;
    private DeliverySlider slider;

    public CommandFactory(Telemetry telemetry, AutoMecanumDriveTrain driveTrain, RollingIntake intake, LimeLight vision, DeliveryPivot pivot, DeliverySlider slider) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY);
    }

    public TurnAngleCommand turnAngle(double angleInDegrees) {
        return new TurnAngleCommand(driveTrain, telemetry, angleInDegrees);
    }

    public MoveSliderCommand extandSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition);
    }

    public MoveSliderCommand collapseSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.CollapsedPosition);
    }

    public TurnAngleRelativeCommand turnAngleRelative(double angleInDegrees) {
        return new TurnAngleRelativeCommand(driveTrain, telemetry, angleInDegrees);
    }

    public InstantCommand waitFor(long timeInMs) {
        return new InstantCommand(() -> Uninterruptibles.sleepUninterruptibly(timeInMs, TimeUnit.MILLISECONDS));
    }

    public MovePivotCommand pivotToInTake() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart);
    }

    public MovePivotCommand pivotToDelivery() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.DeliveryPositionFromStart);
    }

//    public MecanumControllerCommand followTrajectory(Pose2d... waypoints) {
//        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(waypoints), trajectoryConfig);
//        return new MecanumControllerCommand(
//                trajectory,
//                driveTrain::getCurrentPose,
//                driveTrain.getDriveKinematics(),
//                xPIDController,
//                yPIDController,
//                rotPIDController,
//                MAX_VELOCITY,
//                driveTrain::driveWithWheelSpeeds
//        );
//    }

    public InstantCommand stopDriveTrain() {
        return new InstantCommand(() -> driveTrain.stop(), driveTrain);
    }
}
