package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliveryPivot;
import org.firstinspires.ftc.teamcode.subsystems.delivery.DeliverySlider;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.AutoMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake.RollingIntake;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import static org.firstinspires.ftc.teamcode.util.Units.scale;

@SuppressWarnings("unused")
public class CommandFactory {

    private static final PIDController xPIDController = new PIDController(0.015, 0, 0.05);
    private static final PIDController yPIDController = new PIDController(0.015, 0, 0.05);
    private static final ProfiledPIDController rotPIDController = new ProfiledPIDController(0.015, 0, .05, new TrapezoidProfile.Constraints(
            Math.PI * 2,
            Math.PI
    ));

    private static final double MAX_VELOCITY = 400;

    private static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(MAX_VELOCITY, 1);



    private final AutoMecanumDriveTrain driveTrain;
    private final RollingIntake intake;
    private final LimeLight vision;
    private final Telemetry telemetry;
    private final DeliveryPivot pivot;
    private final DeliverySlider slider;

    public CommandFactory(Telemetry telemetry, AutoMecanumDriveTrain driveTrain, RollingIntake intake, LimeLight vision, DeliveryPivot pivot, DeliverySlider slider) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;
    }


    public TelemetryCommand WriteTelemetry() {
        return  new TelemetryCommand(driveTrain, telemetry);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, 20);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double distanceTolerance) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, 1.0, distanceTolerance);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance, long timeOut) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance, timeOut);
    }

    public MoveSliderCommand extendSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition - 100, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSlider(Supplier<Boolean> endHoldingSignalProvider) {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition, DeliverySlider.Direction.EXPANDING).withEndAction(new MoveSliderCommand.EndAction(endHoldingSignalProvider));
    }

    public MoveSliderCommand extendSliderToSpecimen() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderToSpecimenSecondRounnd() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition - 50, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderToDeliverSpecimen() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition + 250, false, DeliverySlider.Direction.EXPANDING, 1500);
    }

    public MoveSliderCommand extendSliderToIntakeSample3() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition + 255, false, DeliverySlider.Direction.EXPANDING, 1500);
    }

    public MoveSliderCommand collapseSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.CollapsedPosition, true, DeliverySlider.Direction.COLLAPSE);
    }

    public SleeperCommand waitFor(long timeInMs) {
        return new SleeperCommand(timeInMs);
    }

    public MovePivotCommand pivotToInTake() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart);
    }

    public MovePivotCommand pivotToSpecimenInTake() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - 450);
    }

    public MovePivotCommand pivotToGroundInTakeBegin() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart + 200);
    }

    public MovePivotCommand pivotToGroundInTakeSample3Begin() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart + 230);
    }

    public MovePivotCommand pivotToStart() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.StartPositionFromStart);
    }


    public MovePivotCommand pivotToDelivery() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.DeliveryPositionFromStart);
    }

    public MovePivotCommand pivotToSpecimenDelivery() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.SpecimenPickupFromStart);

    }

    public SingleRunCommand elbowToIntakePosition() {
        return new SingleRunCommand(intake::SetElbowInIntakePosition);
    }

    public SingleRunCommand elbowToDeliveryPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleDeliveryPosition);
    }

    public SingleRunCommand elbowToStartPosition() {
        return new SingleRunCommand(intake::SetElbowInInStart);
    }

    public SingleRunCommand elbowToSpecimenPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleDeliveryPosition);
    }

    public SmartIntakeCommand intake() {
        return new SmartIntakeCommand(intake);
    }

    public SmartOuttakeCommand outtake() {
        return new SmartOuttakeCommand(intake);
    }

    public IntakeFromGround intakeFromGround() {
        return new IntakeFromGround(intake, pivot, telemetry);
    }

    public MovePivotCommand AutoToGround(int waitTime) {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - scale(600, 0.715), 100, waitTime,  .4);
    }

    public MovePivotCommand AutoToGroundForSample3(int waitTime) {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - scale(650, 0.715), 100, waitTime,  .4);
    }

    public ParallelRaceGroup intakeFromGround2(int waitTime) {
        return new ParallelRaceGroup(
                intake(),
                AutoToGround(waitTime)
        );
    }

    public ParallelCommandGroup intakeFromGroundForSample3(int waitTime) {
        return new ParallelCommandGroup(
                intake(),
                AutoToGroundForSample3(waitTime)
        );
    }

    public IntakeFromWall intakeFromWall() {
        return new IntakeFromWall(driveTrain, intake);
    }

    public SleeperCommand sleep(long timeToSleepMs) {
        return new SleeperCommand(timeToSleepMs);
    }
}
