package org.firstinspires.ftc.teamcode.opmodes.autonomous.command;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.LogCatCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
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
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSlider;
import org.firstinspires.ftc.teamcode.subsystems.specimen.SpecimenSliderClaw;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;
import static org.firstinspires.ftc.teamcode.util.Units.scale;

import android.util.Log;

import java.util.HashMap;
import java.util.Map;

@SuppressWarnings("unused")
public class CommandFactory {

    public static final String LOG_TAG = CommandFactory.class.getSimpleName();
    private static final PIDController xPIDController = new PIDController(0.005, 0, 0.05);
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

    private  final SpecimenSlider specimenSlider;

    private  final SpecimenSliderClaw specimenSliderClaw;

    public CommandFactory(Telemetry telemetry, AutoMecanumDriveTrain driveTrain, RollingIntake intake, LimeLight vision, DeliveryPivot pivot, DeliverySlider slider, SpecimenSlider specimenSlider, SpecimenSliderClaw specimenSliderClaw) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.vision = vision;
        this.telemetry = telemetry;
        this.pivot = pivot;
        this.slider = slider;
        this.specimenSlider = specimenSlider;
        this.specimenSliderClaw = specimenSliderClaw;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard.isEnabled()) {

        }
    }

    public TelemetryCommand WriteTelemetry() {
        return  new TelemetryCommand(driveTrain, telemetry);
    }

    public Command log(String tag, String msg) {
        return new LogCommand(tag, msg);
    }

    public Command logDebug(String msg) {
        return log(CommonConstants.DEBUG_TAG, msg);
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
        return driveToTarget(targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance, timeOut, true);
    }

    public DriveToTargetCommand driveToTarget(double targetX, double targetY, double targetHeading, double minPower, double maxPower, double distanceTolerance, long timeOut, boolean turnOffMotorAtEnd) {
        return new DriveToTargetCommand(driveTrain, telemetry, targetX, targetY, targetHeading, minPower, maxPower, distanceTolerance, timeOut,  turnOffMotorAtEnd);
    }

    public ForwardDistanceCommand checkForwardDistance(double expectedDistance, double minDistance, long timeOut) {
        return new ForwardDistanceCommand(driveTrain, expectedDistance, minDistance, timeOut, telemetry);
    }

    public AlignDriveTrainToSpecimenDelivery alignToSpecimenDelivery(double expectedDistannce, double absoluteMin, long timeOut) {
        return new AlignDriveTrainToSpecimenDelivery(driveTrain, expectedDistannce, absoluteMin, timeOut, telemetry);
    }

    public AlignDriveTrainToSpecimenDelivery alignToSpecimenIntake(double expectedDistanncne, double absoluteMin, long timeOut) {
        return new AlignDriveTrainToSpecimenDelivery(driveTrain, expectedDistanncne, absoluteMin, timeOut, telemetry);
    }

    public ExtendSpecimenSlider extendSpecimenSlider(long timeOut) {
        return new ExtendSpecimenSlider(specimenSlider, timeOut, telemetry);
    }

    public CollapseSpecimenSlider collapseSpecimenSlider(long timeOut) {
        return new CollapseSpecimenSlider(specimenSlider, timeOut, telemetry);
    }

    public InstantCommand openSpecimenClaw() {
        return new InstantCommand(specimenSliderClaw::openClaw, specimenSliderClaw);
    }

    public InstantCommand closeSpecimenClaw() {
        return new InstantCommand(specimenSliderClaw::closeClaw, specimenSliderClaw);
    }

    public MoveSliderCommand extendSlider() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition - 20, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderForOuttake() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition - 20, DeliverySlider.Direction.EXPANDING).withExpandHoldPower(-.2);
    }

//    public MoveSliderCommand extendSlider(Supplier<Boolean> endHoldingSignalProvider) {
//        return new MoveSliderCommand(slider, telemetry, DeliverySlider.BasketDeliveryPosition, DeliverySlider.Direction.EXPANDING).withEndAction(new MoveSliderCommand.EndAction(endHoldingSignalProvider));
//    }

    public MoveSliderCommand extendSliderToSpecimen() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderToSweep() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.ExtendLimit, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderToSpecimenSecondRounnd() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition - 50, DeliverySlider.Direction.EXPANDING);
    }

    public MoveSliderCommand extendSliderToDeliverSpecimen() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition + 250, false, DeliverySlider.Direction.EXPANDING, 1500);
    }

    public MoveSliderCommand extendSliderToIntakeSample3() {
        return new MoveSliderCommand(slider, telemetry, DeliverySlider.StartPosition + 180, false, DeliverySlider.Direction.EXPANDING, 1500);
    }

    public MoveSliderCommand extendSlider(int position) {
        return new MoveSliderCommand(slider, telemetry, position, false, DeliverySlider.Direction.EXPANDING, 1500);
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

    public MovePivotCommand pivotToSweep() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.SweepPositionFromStart - 100);
    }

    public MovePivotCommand pivotToJustAboveSweep() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.SweepPositionFromStart + 400);
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

    public MovePivotCommand pivotTo(int position) {
        return new MovePivotCommand(pivot, telemetry, position);
    }

    public MovePivotCommand pivotToIntakeRetry() {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - 200);
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

    public Command elbowToPosition(double position) {
        return new SingleRunCommand(() -> intake.setElbowToPosition(position));
    }

    public SingleRunCommand elbowToIntakePositionForSample3() {
        return new SingleRunCommand(intake::SetElbowInIntakePositionForSample3);
    }

    public SingleRunCommand elbowToDeliveryPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleDeliveryPosition);
    }

    public SingleRunCommand elbowToSweepPosition() {
        return new SingleRunCommand(intake::SetElbowInSampleSweepPosition);
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
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - scale(650, 0.715), 100, waitTime,  .07);
    }

    public MovePivotCommand AutoToGroundForSample(double target, int waitTime) {
        return new MovePivotCommand(pivot, telemetry, target, 300, waitTime,  .02);
    }

    public MovePivotCommand AutoToGroundForSample3(int waitTime) {
        return new MovePivotCommand(pivot, telemetry, DeliveryPivot.IntakePositionFromStart - scale(680, 0.715), 100, waitTime,  .07);
    }

    public ParallelRaceGroup intakeFromGround2(int waitTime) {
        return new ParallelRaceGroup(
                intake(),
                AutoToGround(waitTime)
        );
    }

    public Command intakeFromGroundForSample3(int waitTime) {
        return new ParallelRaceGroup(
                intake(),
                AutoToGroundForSample3(waitTime)
        );
    }

    public Command intakeFromGround(double groundTarget, int waitTime) {
        return new ParallelRaceGroup(
                intake(),
                AutoToGroundForSample(groundTarget, waitTime)
        );
    }

    public IntakeFromWall intakeFromWall() {
        return new IntakeFromWall(driveTrain, intake);
    }

    public SleeperCommand sleep(long timeToSleepMs) {
        return new SleeperCommand(timeToSleepMs);
    }

    public Command inCaseSampleIntakeFailed(String sampleName, Command command) {
        Map<Object, Command> commandMap = new HashMap<>();
        commandMap.put(true, doNothing().alongWith(new LogCatCommand(LOG_TAG, ">>>>> " + sampleName + " in place, do nothing", Log.INFO)));
        commandMap.put(false, command.alongWith(new LogCatCommand(LOG_TAG, ">>>>> " + sampleName + " in take failed, retrying...", Log.INFO)));
        return new SelectCommand(commandMap, intake::IsSampleIntaken);
    }
    public Command doNothing() {
        return new InstantCommand();
    }
}
