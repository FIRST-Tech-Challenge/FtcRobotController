package org.firstinspires.ftc.teamcode.autoutils;

import static java.sql.Types.OTHER;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.WristPositionCommand;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FingerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.util.Other.ArrayTypeValue;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Other;

import java.util.HashMap;

/** @noinspection rawtypes*/
public class CompTrajectoryGenerator {

    private final SampleMecanumDrive DRIVE;

    /* Pre-made Trajectories */
    public final TrajectorySequence BLUE_BOTTOM, BLUE_BOTTOM_PARK;
    public final TrajectorySequence BLUE_TOP, BLUE_TOP_PARK;
    public final TrajectorySequence RED_BOTTOM, RED_BOTTOM_PARK;
    public final TrajectorySequence RED_TOP, RED_TOP_PARK;

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

    final HashMap<Other, DynamicTypeValue> OTHER;

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
        this.OTHER = other;

        this.BLUE_BOTTOM = generateFieldTrajectoryNew(trajectories.BLUE_BOTTOM, other, park_left);
        this.BLUE_TOP = generateFieldTrajectoryNew(trajectories.BLUE_TOP, other, park_left);
        this.RED_BOTTOM = generateFieldTrajectoryNew(trajectories.RED_BOTTOM, other, park_left);
        this.RED_TOP = generateFieldTrajectoryNew(trajectories.RED_TOP, other, park_left);

        this.BLUE_BOTTOM_PARK = generateFieldTrajectoryNewPark(trajectories.BLUE_BOTTOM, other, park_left);
        this.BLUE_TOP_PARK = generateFieldTrajectoryNewPark(trajectories.BLUE_TOP, other, park_left);
        this.RED_BOTTOM_PARK = generateFieldTrajectoryNewPark(trajectories.RED_BOTTOM, other, park_left);
        this.RED_TOP_PARK = generateFieldTrajectoryNewPark(trajectories.RED_TOP, other, park_left);
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
    /*public TrajectorySequence generateFieldTrajectory(@NonNull final trajectories trajectory,
                                                      final HashMap<Other, DynamicTypeValue> other,
                                                      final boolean PARK_LEFT) {
        final TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(25);
        final TrajectoryVelocityConstraint velocityConstraint =
                SampleMecanumDrive.getVelocityConstraint(30,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

        final double WAIT_TIME = 2.4288;

        switch (trajectory) {
            case BLUE_BOTTOM:
                if (PARK_LEFT) {  
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70, Math.toRadians(0)))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, 66))
                        .lineTo(new Vector2d(50, 66))
                        .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70, Math.toRadians(270)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .waitSeconds(WAIT_TIME)
                            .lineTo(new Vector2d(-36, 10))
                            .turn(Math.toRadians(90))
                            .lineTo(new Vector2d(58, 23))
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
    }*/

    int wallDistance = 48;
    final double WAIT_TIME = 2.4288;
    final int WALL_CENTER = 42;

    public TrajectorySequence generateFieldTrajectoryNew(@NonNull final trajectories trajectory,
                                                      final HashMap<Other, DynamicTypeValue> other,
                                                         boolean PARK_LEFT) {
        final TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(25);
        final TrajectoryVelocityConstraint velocityConstraint =
                SampleMecanumDrive.getVelocityConstraint(30,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

        int yScale = (trajectory == trajectories.RED_TOP || trajectory == trajectories.RED_BOTTOM) ? -1 : 1;
        // TODO: CHECK IF THIS WORKS
        if (yScale == -1) {
            // The code below is based on blue park
            PARK_LEFT = !PARK_LEFT;
        }

        // Blue points down (on the graph https://www.desmos.com/calculator/cv0s9vq08g)
        // which is 270 on a unit circle
        double forwardAngle = Math.toRadians(270);
        if (yScale == -1) {
            // Red points up (on the graph)
            // which is 90 on a unit circle
            forwardAngle = Math.toRadians(90);
        }

        switch (trajectory) {
            case BLUE_BOTTOM:
            case RED_BOTTOM:
                // ADD GRAPHS FROM https://www.desmos.com/calculator/bnm9pzy7p5 TO HERE (AND REMEMBER TO MULTIPLY ANGLES/Y VALUES BY yScale)
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70*yScale, Math.toRadians(0)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .waitSeconds(WAIT_TIME)
                            .lineTo(new Vector2d(-36, 66*yScale))
                            .lineTo(new Vector2d(wallDistance, 66*yScale))
                            .lineTo(new Vector2d(wallDistance, WALL_CENTER*yScale))
                            // maybe swap the rotation and the move so we dont hit the board
                            .turn(Math.toRadians(180))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(-36, 70*yScale, forwardAngle))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, 10*yScale))
                        .turn(Math.toRadians(90*yScale))
                        .lineTo(new Vector2d(wallDistance, 10*yScale))
                        .lineTo(new Vector2d(wallDistance, WALL_CENTER*yScale))
                        .turn(Math.toRadians(180))
                        .build();
            case BLUE_TOP:
            case RED_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(new Pose2d(12, 70*yScale, Math.toRadians(0)))
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .waitSeconds(WAIT_TIME)
                            .lineTo(new Vector2d(12, 66*yScale))
                            .lineTo(new Vector2d(wallDistance, 66*yScale))
                            .lineTo(new Vector2d(wallDistance, WALL_CENTER*yScale))
                            // maybe put this line \/ above this line /\
                            .turn(Math.toRadians(180))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(new Pose2d(12, 70*yScale, forwardAngle))
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, WALL_CENTER*yScale))
                        // maybe make this negative to reduce unnessesary turning
                        // .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90*yScale))
                        .lineTo(new Vector2d(wallDistance, WALL_CENTER*yScale))
                        // if made the other one negative, remove this line
                        .turn(Math.toRadians(180))
                        .build();
            default:
                return null;
        }
    }

    public TrajectorySequence generateFieldTrajectoryNewPark(@NonNull final trajectories trajectory,
                                                         final HashMap<Other, DynamicTypeValue> other,
                                                         final boolean PARK_LEFT) {
        final TrajectoryAccelerationConstraint accelerationConstraint =
                SampleMecanumDrive.getAccelerationConstraint(25);
        final TrajectoryVelocityConstraint velocityConstraint =
                SampleMecanumDrive.getVelocityConstraint(30,
                        DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH);

        int yScale = (trajectory == trajectories.RED_TOP || trajectory == trajectories.RED_BOTTOM) ? -1 : 1;

        Pose2d startPosition = new Pose2d(wallDistance, WALL_CENTER*yScale, Math.toRadians(180));
        int distanceFactor = 8;
        int left = 60-distanceFactor;
        int right = 16+distanceFactor;
        if (yScale == -1) {
            right = 60-distanceFactor;
            left = 16+distanceFactor;
        }

        if (PARK_LEFT) {
            return DRIVE.trajectorySequenceBuilder(startPosition)
                    .setConstraints(velocityConstraint, accelerationConstraint)
                    .lineTo(new Vector2d(wallDistance, left * yScale))
                    .build();
        } else {
            return DRIVE.trajectorySequenceBuilder(startPosition)
                    .setConstraints(velocityConstraint, accelerationConstraint)
                    .lineTo(new Vector2d(wallDistance, right * yScale))
                    .build();
        }

        /*switch (trajectory) {
            case BLUE_BOTTOM:
            case RED_BOTTOM:
                // ADD GRAPHS FROM https://www.desmos.com/calculator/bnm9pzy7p5 TO HERE (AND REMEMBER TO MULTIPLY ANGLES/Y VALUES BY yScale)
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(startPosiiton)
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .waitSeconds(WAIT_TIME)
                            .lineTo(new Vector2d(-36, 66 * yScale))
                            .lineTo(new Vector2d(wallDistance, 66 * yScale))
                            .lineTo(new Vector2d(wallDistance, 38 * yScale))
                            .turn(Math.toRadians(180))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(startPosiiton)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(-36, 10 * yScale))
                        .turn(Math.toRadians(90 * yScale))
                        .lineTo(new Vector2d(wallDistance, 10 * yScale))
                        .lineTo(new Vector2d(wallDistance, 38 * yScale))
                        .turn(Math.toRadians(180))
                        .build();
            case BLUE_TOP:
            case RED_TOP:
                if (PARK_LEFT) {
                    return DRIVE.trajectorySequenceBuilder(startPosiiton)
                            .setConstraints(velocityConstraint, accelerationConstraint)
                            .waitSeconds(WAIT_TIME)
                            .lineTo(new Vector2d(12, 66 * yScale))
                            .lineTo(new Vector2d(wallDistance, 66 * yScale))
                            .lineTo(new Vector2d(wallDistance, 38 * yScale))
                            .turn(Math.toRadians(180))
                            .build();
                }
                return DRIVE.trajectorySequenceBuilder(startPosiiton)
                        .setConstraints(velocityConstraint, accelerationConstraint)
                        .waitSeconds(WAIT_TIME)
                        .lineTo(new Vector2d(12, 38 * yScale))
                        .turn(Math.toRadians(90 * yScale))
                        .lineTo(new Vector2d(wallDistance, 38 * yScale))
                        .turn(Math.toRadians(180))
                        .build();
            default:
                return null;
        }*/
    }

    private enum PlacePixelState {
        START,
        ROTATE_ARM_WRIST,
        ARM_ADJUSTMENT,
        RELEASE_FINGERS,
        RESET_ARM, // optional. can be used to move the arm down if we need to
        FINISH
    }

    public void placePixelOnBoard() {
        // TODO: test if placing pixel on board works
        boolean running = true;
        boolean keepFingerPinched = true;

        HashMap<RobotHardwareInitializer.Arm, DcMotor> arms = new HashMap<>(2);
        ArrayTypeValue<DcMotor> armArray = (ArrayTypeValue<DcMotor>) OTHER.get(Other.ARM);

        arms.put(RobotHardwareInitializer.Arm.ARM1, armArray.get(0));
        arms.put(RobotHardwareInitializer.Arm.ARM2, armArray.get(1));

        // Zero out the wrist
        WristSubsystem wristSubsystem = new WristSubsystem((DcMotorEx) OTHER.get(Other.WRIST).getValue(), false);
        wristSubsystem.zero();

        ArmSubsystem armSubsystem = new ArmSubsystem(arms);
        FingerSubsystem fingerSubsystem = new FingerSubsystem((Servo) OTHER.get(Other.FINGER).getValue());

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
                    // Pinch fingers for half a second
                    if (elapsedTime.seconds() > .5) {
                        state = PlacePixelState.ROTATE_ARM_WRIST;
                        elapsedTime.reset();
                    }
                    break;
                case ROTATE_ARM_WRIST:
                    // Move the arm to the back of the board

                    // Move wrist to the board position slowly
                    if (elapsedTime.seconds() >= .6) {
                        wristSubsystem.setWristPosition(WristPositionCommand.getBoardTargetPosition(), .5f);
                    }

                    armSubsystem.positionMoveArm(ArmSubsystem.ArmPosition.BOARD.getPosition());

                    //boolean atBack = //armSubsystem.getArmMotor1().getCurrentPosition() >= ArmSubsystem.ArmPosition.BOARD.getPosition();
                    boolean atBack = !(armSubsystem.getArmMotor1().isBusy());

                    // Check if it got to the board
                    // It reached the board

                    // If the arm is positioned at the back of the board, release the fingers
                    if (atBack) {
                        state = PlacePixelState.ARM_ADJUSTMENT;
                        elapsedTime.reset();
                    }
                    break;
                case ARM_ADJUSTMENT:
                    // im sorry for this
                    while (elapsedTime.seconds() < .8) {
                        armSubsystem.manualMoveArm(.555);
                    }
                    armSubsystem.manualMoveArm(0);

                    state = PlacePixelState.RELEASE_FINGERS;
                    elapsedTime.reset();
                    break;
                case RELEASE_FINGERS:
                    // Drop the pixels on the board
                    keepFingerPinched = false;

                    // After the pixels have had time to drop, begin to parks
                    if (elapsedTime.milliseconds() > 1500) {
                        state = PlacePixelState.RESET_ARM;
                        elapsedTime.reset();
                    }
                    break;
                case RESET_ARM:
                    // optionally move the arm down a bit more so its not in a weird position
                    while (elapsedTime.seconds() < 1) {
                        armSubsystem.manualMoveArm(-1);
                    }
                    armSubsystem.manualMoveArm(0);
                    elapsedTime.reset();
                    state = PlacePixelState.FINISH;
                    break;
                case FINISH:
                    running = false;
                    break;
            }
        }
    }
}