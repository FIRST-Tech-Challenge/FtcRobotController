package org.firstinspires.ftc.teamcode.autoutils;

import static org.firstinspires.ftc.teamcode.autoutils.OtherAutoUtils.DefinedLocations.BLUE_BOARD;
import static org.firstinspires.ftc.teamcode.autoutils.OtherAutoUtils.DefinedLocations.RED_BOARD;
import static org.firstinspires.ftc.teamcode.autoutils.OtherAutoUtils.Quadrants;
import static org.firstinspires.ftc.teamcode.autoutils.OtherAutoUtils.avoidTruss;
import static org.firstinspires.ftc.teamcode.autoutils.OtherAutoUtils.getCurrentQuadrant;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.WristPositionCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.FTCDashboardPackets;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Other;

import java.util.HashMap;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/** @noinspection rawtypes*/
public class CompTrajectoryGenerator {

    private final SampleMecanumDrive DRIVE;
    private final HashMap<Other, DynamicTypeValue> OTHER;
    private final FTCDashboardPackets dbp = new FTCDashboardPackets("TrajectoryGen");

    private final String TEAM_PROP_LABEL = "Cube";

    /* Pre-made Trajectories */
    public final TrajectorySequence BLUE_BOTTOM;
    public final TrajectorySequence BLUE_TOP;
    public final TrajectorySequence RED_BOTTOM;
    public final TrajectorySequence RED_TOP;

    /**
     * The public enum that stores the current valid trajectories
     * @author Carter Rommelfanger
     */
    public enum trajectories {
        BLUE_BOTTOM,
        BLUE_TOP,
        RED_BOTTOM,
        RED_TOP
    }

    private final TrajectoryAccelerationConstraint accelerationConstraint =
            SampleMecanumDrive.getAccelerationConstraint(25);
    private final TrajectoryVelocityConstraint velocityConstraint =
            SampleMecanumDrive.getVelocityConstraint(30,
                    DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

    /**
     * Generates pre-made sequences to be used in autonomous mode
     * @param drive The pre-initialized SampleMecanumDrive class to be used
     * @param other The hashmap of all the other robot parts (Hand, Arm, Intake, etc.)
     * @author Carter Rommelfanger
     */
    public CompTrajectoryGenerator(SampleMecanumDrive drive,
                                   final HashMap<Other, DynamicTypeValue> other,
                                   final boolean park_left) {
        DRIVE = drive;

        dbp.createNewTelePacket();

        this.BLUE_BOTTOM = generateFieldTrajectory(trajectories.BLUE_BOTTOM, other, park_left);
        this.BLUE_TOP = generateFieldTrajectory(trajectories.BLUE_TOP, other, park_left);
        this.RED_BOTTOM = generateFieldTrajectory(trajectories.RED_BOTTOM, other, park_left);
        this.RED_TOP = generateFieldTrajectory(trajectories.RED_TOP, other, park_left);

        OTHER = other;
    }

    /**
     * Sets the pose estimate of the current mecanum drive to the inputted trajectory
     * @param trajectory The trajectory to set the pose estimate to
     * @author Carter Rommelfanger
     */
    public void setStartingPosition(@NonNull trajectories trajectory) {
        Pose2d pose;
        switch (trajectory) {
            case BLUE_BOTTOM:
                pose = BLUE_BOTTOM.start();
                break;

            case BLUE_TOP:
                pose = BLUE_TOP.start();
                break;

            case RED_BOTTOM:
                pose = RED_BOTTOM.start();
                break;

            case RED_TOP:
                pose = RED_TOP.start();
                break;

            default:
                return;
        }

        DRIVE.setPoseEstimate(pose);
    }

    /**
     * Initializes a pre-made field trajectory based off the given trajectory enum
     * @param trajectory The trajectory to initialize
     * @param other The hashmap of all the other robot parts (Hand, Arm, Intake, etc.)
     * @param PARK_LEFT Whether to have the trajectory sequence return the sequence for
     *                  parking on the left or right side of the field
     * @return The initialized trajectory if the given trajectory name exists,
     *         otherwise returns null\
     * @author Carter Rommelfanger
     */
    public TrajectorySequence generateFieldTrajectory(@NonNull final trajectories trajectory,
                                                      final HashMap<Other, DynamicTypeValue> other,
                                                      final boolean PARK_LEFT) {
        final double WAIT_TIME = 2.4288;

        switch (trajectory) {
            case BLUE_BOTTOM:
                if (!PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70, Math.toRadians(270)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, 10))
                        .turn(Math.toRadians(90))
                        .lineTo(new Vector2d(58, 23))
                        .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70, Math.toRadians(270)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, 66))
                        .lineTo(new Vector2d(50, 66))
                        .build();

            case BLUE_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(12, 70, Math.toRadians(0)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, 66))
                        .lineTo(new Vector2d(60, 66))
                        .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12, 70, Math.toRadians(270)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, 27))
                        .lineTo(new Vector2d(50, 27))
                        .build();

            case RED_BOTTOM:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, -70, Math.toRadians(90)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, -20))
                        .turn(Math.toRadians(-90))
                        .lineTo(new Vector2d(58, -20))
                        .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, -70, Math.toRadians(0)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, -66))
                        .lineTo(new Vector2d(50, -66))
                        .build();

            case RED_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(12,  -70, Math.toRadians(90)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, -15))
                        .turn(Math.toRadians(-90))
                        .lineTo(new Vector2d(60, -15))
                        .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12, -70, Math.toRadians(0)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, -66))
                        .lineTo(new Vector2d(60, -66))
                        .build();

            default:
                return null;
        }
    }

    public enum PixelStates {
        DETECT_MIDDLE,
        DETECT_LEFT,
        DETECT_RIGHT,
        PLACE_LEFT,
        PLACE_MIDDLE,
        PLACE_RIGHT,
        POOP_PIXEL,
        MOVE_BOARD,
        PLACE_BOARD_LEFT,
        PLACE_BOARD_MIDDLE,
        PLACE_BOARD_RIGHT,
        PARK,
        IDLE
    }

    public Vector2d pose2dToVector(final Pose2d pose) {
        return new Vector2d(pose.getX(), pose.getY());
    }

    /**
     * Outputs a Pose2d that adds or subtracts from the starting Pose2d to move the robot left
     * @param startingPose the starting pose to modify
     * @param distance the distance to move the robot left
     *                 can be negative to go right
     * @return a modified Pose2d that moves the robot left
     */
    public Pose2d moveX(final Pose2d startingPose, final double distance) {
        switch ((int) Math.toRadians(startingPose.getHeading())) {
            case 270:
                return new Pose2d(
                        startingPose.getX() - distance,
                        startingPose.getY(),
                        startingPose.getHeading()
                );
            case 90:
                return new Pose2d(
                        startingPose.getX() + distance,
                        startingPose.getY(),
                        startingPose.getHeading()
                );
            case 0:
                return new Pose2d(
                        startingPose.getX(),
                        startingPose.getY() + distance,
                        startingPose.getHeading()
                );
            case 180:
                return new Pose2d(
                        startingPose.getX(),
                        startingPose.getY() - distance,
                        startingPose.getHeading()
                );
            default:
                return startingPose;
        }
    }

    /**
     * Outputs a Vector2d that adds or subtracts from the starting Pose2d to move the robot left
     * @param startingPose the starting pose to modify
     * @param distance the distance to move the robot left
     *                 can be negative to go right
     * @return a Vector2d that moves the robot left
     */
    public Vector2d vector_moveX(final Pose2d startingPose, final double distance) {
        switch ((int) Math.toRadians(startingPose.getHeading())) {
            case 270:
                return new Vector2d(
                        startingPose.getX() - distance,
                        startingPose.getY()
                );
            case 90:
                return new Vector2d(
                        startingPose.getX() + distance,
                        startingPose.getY()
                );
            case 0:
                return new Vector2d(
                        startingPose.getX(),
                        startingPose.getY() + distance
                );
            case 180:
                return new Vector2d(
                        startingPose.getX(),
                        startingPose.getY() - distance
                );
            default:
                return pose2dToVector(startingPose);
        }
    }

    /**
     * Outputs a Pose2d that adds or subtracts from the starting Pose2d to move the robot forward
     * @param startingPose the starting pose to modify
     * @param distance the distance to move the robot forward
     *                 can be negative to go backward
     * @return a modified Pose2d that moves the robot forward
     */
    public Pose2d moveY(final Pose2d startingPose, final double distance) {
        switch ((int) Math.toRadians(startingPose.getHeading())) {
            case 270:
                return new Pose2d(
                        startingPose.getX(),
                        startingPose.getY() - distance,
                        startingPose.getHeading()
                );
            case 90:
                return new Pose2d(
                        startingPose.getX(),
                        startingPose.getY() + distance,
                        startingPose.getHeading()
                );
            case 0:
                return new Pose2d(
                        startingPose.getX() + distance,
                        startingPose.getY(),
                        startingPose.getHeading()
                );
            case 180:
                return new Pose2d(
                        startingPose.getX() - distance,
                        startingPose.getY(),
                        startingPose.getHeading()
                );
            default:
                return startingPose;
        }
    }

    /**
     * Outputs a Pose2d that adds or subtracts from the starting Pose2d to move the robot forward
     * @param startingPose the starting pose to modify
     * @param distance the distance to move the robot forward
     *                 can be negative to go backward
     * @return a modified Pose2d that moves the robot forward
     */
    public Vector2d vector_moveY(final Pose2d startingPose, final double distance) {
        switch ((int) Math.toRadians(startingPose.getHeading())) {
            case 270:
                return new Vector2d(
                        startingPose.getX(),
                        startingPose.getY() - distance
                );
            case 90:
                return new Vector2d(
                        startingPose.getX(),
                        startingPose.getY() + distance
                );
            case 0:
                return new Vector2d(
                        startingPose.getX() + distance,
                        startingPose.getY()
                );
            case 180:
                return new Vector2d(
                        startingPose.getX() - distance,
                        startingPose.getY()
                );
            default:
                return pose2dToVector(startingPose);
        }
    }

    public TrajectorySequence moveToPixelBoard(final Pose2d startingPose,
                                               final boolean isBlueTeam) {
        final Quadrants QUADRANT = getCurrentQuadrant(startingPose);

        switch (QUADRANT) {
            case BLUE_TOP:
                return DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(avoidTruss(startingPose))
                        .lineTo(BLUE_BOARD.getLocation())
                        .build();
            case BLUE_BOTTOM:
                return DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(avoidTruss(startingPose))
                        .lineTo(new Vector2d(-36, 10))
                        .turn(90)
                        .build();
            case RED_TOP:
                return DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(avoidTruss(startingPose))
                        .lineTo(RED_BOARD.getLocation())
                        .build();
            case RED_BOTTOM:
                return DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(avoidTruss(startingPose))
                        .lineTo(new Vector2d(-36, -20))
                        .turn(Math.toRadians(-90))
                        .build();
        }

        return null;
    }

    public HashMap<PixelStates, TrajectorySequence> purple_pixel(Pose2d startingPose) {
        HashMap<PixelStates, TrajectorySequence> out = new HashMap<>();

        final double DETECT_DISTANCE = 2;
        final double PLACE_DISTANCE = 1;

        final TrajectorySequence PLACE_MIDDLE =
                DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveY(startingPose, PLACE_DISTANCE))
                        .build();

        final TrajectorySequence DETECT_LEFT =
                DRIVE.trajectorySequenceBuilder(startingPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveX(startingPose, DETECT_DISTANCE))
                        .waitSeconds(5)
                        .build();

        final TrajectorySequence PLACE_LEFT =
                DRIVE.trajectorySequenceBuilder(DETECT_LEFT.end())
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveY(DETECT_LEFT.end(), PLACE_DISTANCE))
                        .build();

        final TrajectorySequence DETECT_RIGHT =
                DRIVE.trajectorySequenceBuilder(DETECT_LEFT.end())
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveX(DETECT_LEFT.end(), -2 * DETECT_DISTANCE))
                        .waitSeconds(5)
                        .build();

        final TrajectorySequence PLACE_RIGHT =
                DRIVE.trajectorySequenceBuilder(DETECT_RIGHT.end())
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveY(DETECT_RIGHT.end(), PLACE_DISTANCE))
                        .build();

        // Add all of the Trajectory Sequences
        out.put(PixelStates.PLACE_MIDDLE, PLACE_MIDDLE);
        out.put(PixelStates.DETECT_LEFT, DETECT_LEFT);
        out.put(PixelStates.PLACE_LEFT, PLACE_LEFT);
        out.put(PixelStates.DETECT_RIGHT, DETECT_RIGHT);
        out.put(PixelStates.PLACE_RIGHT, PLACE_RIGHT);


        return out;
    }

    public TrajectorySequence moveToPlacePixelOnBoard(
            final Pose2d currentPose,
            final PixelStates currentState,
            final SpikeMarkPlacement spikeMarkPlacement) {

        switch (currentState) {
            case PLACE_BOARD_LEFT:
                return DRIVE.trajectorySequenceBuilder(currentPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveY(currentPose, 5))
                        .build();
            case PLACE_BOARD_RIGHT:
                return DRIVE.trajectorySequenceBuilder(currentPose)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .lineTo(vector_moveY(currentPose, -5))
                        .build();
        }

        return null;
    }

    private enum PlacePixelState {
        START,
        ROTATE_ARM_WRIST,
        RELEASE_FINGERS,
        RESET_ARM, // optional. can be used to move the arm down if we need to
        FINISH
    }

    public void placePixelOnBoard() {
        // TODO: test if placing pixel on board works
        boolean running = true;
        boolean keepFingerPinched = true;

        HashMap<RobotHardwareInitializer.Arm, DcMotor> arms = new HashMap<>(2);
        DcMotor[] armArray = (DcMotor[]) OTHER.get(Other.ARM).getValue();
        arms.put(RobotHardwareInitializer.Arm.ARM1, armArray[0]);
        arms.put(RobotHardwareInitializer.Arm.ARM2, armArray[0]);

        WristSubsystem wristSubsystem = new WristSubsystem((DcMotorEx) OTHER.get(Other.WRIST).getValue(), false);
        ArmSubsystem armSubsystem = new ArmSubsystem(arms);
        FingerSubsystem fingerSubsystem = new FingerSubsystem((Servo) OTHER.get(Other.WRIST).getValue());

        ElapsedTime elapsedTime = new ElapsedTime();
        PlacePixelState state = PlacePixelState.START;
        while (running) {

            if (keepFingerPinched) {
                fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.CLOSED);
            } else {
                fingerSubsystem.locomoteFinger(FingerSubsystem.FingerPositions.OPEN);
            }

            switch (state) {
                case START:
                    // Pinch Fingers
                    keepFingerPinched = true;
                    state = PlacePixelState.ROTATE_ARM_WRIST;
                    elapsedTime.reset();
                    break;
                case ROTATE_ARM_WRIST:
                    // Move wrist to the board position
                    wristSubsystem.setWristPosition(WristPositionCommand.getBoardTargetPosition());

                    // Move the arm to the back of the board
                    armSubsystem.manualMoveArm(1);
                    int targetPosition = ArmSubsystem.positionFromAngle(armSubsystem.getArmMotor1(), ArmSubsystem.ArmPosition.BOARD.getPosition(), AngleUnit.DEGREES);

                    boolean atBack = false;

                    // Check if it got to the board
                    if (armSubsystem.getArmMotor1().getCurrentPosition() >= targetPosition) {
                        // It reached the board
                        atBack = true;
                    }

                    // If the arm is positioned at the back of the board, release the fingers
                    if (atBack) {
                        state = PlacePixelState.RELEASE_FINGERS;
                        elapsedTime.reset();
                    }
                    break;
                case RELEASE_FINGERS:
                    // Drop the pixels on the board
                    keepFingerPinched = false;

                    // After the pixels have had time to drop, begin to parks
                    if (elapsedTime.milliseconds() > 400) {
                        state = PlacePixelState.RESET_ARM;
                        elapsedTime.reset();
                    }
                    break;
                case RESET_ARM:
                    // optionally move the arm down a bit more so its not in a weird position
                    state = PlacePixelState.FINISH;
                    break;
                case FINISH:
                    running = false;
                    break;
            }
        }
    }

    private enum SpikeMarkPlacement {
        LEFT,
        MIDDLE,
        RIGHT,
        ERROR
    }

    public static SpikeMarkPlacement getSpikeMarkPlacementFromCurrentState(
            final PixelStates currentState) {
        switch (currentState) {
            case PLACE_LEFT:
                return SpikeMarkPlacement.LEFT;
            case PLACE_MIDDLE:
                return SpikeMarkPlacement.MIDDLE;
            case PLACE_RIGHT:
                return SpikeMarkPlacement.RIGHT;
            default:
                return SpikeMarkPlacement.ERROR;
        }
    }

    public void runPurplePixelDetection(final Pose2d startingPose,
                                        final boolean isBlueTeam,
                                        final boolean parkLeft,
                                        BooleanSupplier opModeIsActive,
                                        BooleanSupplier isStopRequested) {
        PixelStates state = PixelStates.DETECT_MIDDLE;

        final HashMap<PixelStates, TrajectorySequence> PURPLE_PIXEL_SEQUENCES =
                purple_pixel(startingPose);

        final ObjectDetector OBJ_DETECTOR = new ObjectDetector(OTHER);

        SpikeMarkPlacement spikeMarkPlacement = SpikeMarkPlacement.ERROR;
        Pose2d lastPose;

        //

        while (opModeIsActive.getAsBoolean() && !isStopRequested.getAsBoolean()) {
            if (state != null) {
                System.out.println("Current state: "+state.name());
            }
            switch (state) {
                case DETECT_MIDDLE:
                    dbp.info("In Detect Middle", true);
                    state = PixelStates.PLACE_MIDDLE;
                    // TODO: uncomment this and fix object detection
                    //state = (OBJ_DETECTOR.isObjectRecognized(TEAM_PROP_LABEL)) ?
                    //                PixelStates.PLACE_MIDDLE : PixelStates.DETECT_LEFT;
                    dbp.info((OBJ_DETECTOR.isObjectRecognized(TEAM_PROP_LABEL)) ?
                            "Detected Team Prop" : "Did not detect team prop...", true);
                    break;
                case DETECT_LEFT:
                    dbp.info("In Detect Left", true);
                    DRIVE.followTrajectorySequence(
                            Objects.requireNonNull(PURPLE_PIXEL_SEQUENCES.get(state))
                    );
                    state = (OBJ_DETECTOR.isObjectRecognized(TEAM_PROP_LABEL)) ?
                            PixelStates.PLACE_LEFT : PixelStates.DETECT_RIGHT;
                    dbp.info((OBJ_DETECTOR.isObjectRecognized(TEAM_PROP_LABEL)) ?
                            "Detected Team Prop" : "Did not detect team prop...", true);
                    break;
                case DETECT_RIGHT:
                    dbp.info("In Detect Right", true);
                    DRIVE.followTrajectorySequence(
                            Objects.requireNonNull(PURPLE_PIXEL_SEQUENCES.get(state))
                    );
                    if (OBJ_DETECTOR.isObjectRecognized(TEAM_PROP_LABEL)) {
                        dbp.info("Detected team prop", true);
                        state = PixelStates.PLACE_RIGHT;
                    }
                    else {
                        dbp.info("Did not detect prop");
                        state = PixelStates.PARK;
                    }
                    break;
                case PLACE_MIDDLE:
                case PLACE_LEFT:
                case PLACE_RIGHT:
                    DRIVE.followTrajectorySequence(
                            Objects.requireNonNull(PURPLE_PIXEL_SEQUENCES.get(state))
                    );
                    state = PixelStates.POOP_PIXEL;
                    spikeMarkPlacement = getSpikeMarkPlacementFromCurrentState(state);
                    lastPose = Objects.requireNonNull(PURPLE_PIXEL_SEQUENCES.get(state)).start();
                    break;
                case POOP_PIXEL:
                    final ServoEx SERVO = (ServoEx) OTHER.get(Other.PIXEL_POOPER);
                    assert SERVO != null;
                    SERVO.turnToAngle(300);
                    state = PixelStates.MOVE_BOARD;
                    break;
                case MOVE_BOARD:
                    DRIVE.followTrajectorySequence(
                            moveToPixelBoard(DRIVE.getPoseEstimate(), isBlueTeam)
                    );
                    switch (spikeMarkPlacement) {
                        case LEFT:
                            state = PixelStates.PLACE_BOARD_LEFT;
                            break;
                        case MIDDLE:
                            state = PixelStates.PLACE_BOARD_MIDDLE;
                            break;
                        case RIGHT:
                            state = PixelStates.PLACE_BOARD_RIGHT;
                            break;
                        default:
                            state = PixelStates.PARK;
                    }
                    break;
                case PLACE_BOARD_MIDDLE:
                    placePixelOnBoard();
                    state = PixelStates.PARK;
                    break;
                case PLACE_BOARD_LEFT:
                case PLACE_BOARD_RIGHT:
                    lastPose = DRIVE.getPoseEstimate();
                    DRIVE.followTrajectorySequence(
                            moveToPlacePixelOnBoard(lastPose, state, spikeMarkPlacement)
                    );
                    placePixelOnBoard();

                    state = PixelStates.PARK;
                    break;
                case PARK:
                    // TODO: implement parking
                    return;
                case IDLE:
                    return;
                default:
                    break;
            }
        }
    }
}
