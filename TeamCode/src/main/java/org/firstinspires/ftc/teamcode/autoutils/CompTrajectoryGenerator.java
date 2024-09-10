package org.firstinspires.ftc.teamcode.autoutils;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Other.DynamicTypeValue;
import org.firstinspires.ftc.teamcode.util.RobotHardwareInitializer.Other;

import java.util.HashMap;

/** @noinspection rawtypes*/
@Deprecated
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

    int wallDistance = 47;
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
                        .lineTo(new Vector2d(-36, 19*yScale))
                        .turn(Math.toRadians(90*yScale))
                        .lineTo(new Vector2d(wallDistance, 19*yScale))
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
        int distanceFactor = 0;
        int left = 65-distanceFactor;
        int right = 22+distanceFactor;
        if (yScale == -1) {
            // swap right and left for red
            right = 65-distanceFactor;
            left = 22+distanceFactor;
        }

        final int DISTANCE_FROM_WALL = 8;

        if (PARK_LEFT) {
            return DRIVE.trajectorySequenceBuilder(startPosition)
                    .setConstraints(velocityConstraint, accelerationConstraint)
                    .lineTo(new Vector2d(wallDistance, left * yScale))
                    .lineTo(new Vector2d(wallDistance + DISTANCE_FROM_WALL, left * yScale))
                    .build();
        } else {
            return DRIVE.trajectorySequenceBuilder(startPosition)
                    .setConstraints(velocityConstraint, accelerationConstraint)
                    .lineTo(new Vector2d(wallDistance, right * yScale))
                    .lineTo(new Vector2d(wallDistance + DISTANCE_FROM_WALL, right * yScale))
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

    @Deprecated
    private enum PlacePixelState {
        START,
        ROTATE_ARM_WRIST,
        ARM_ADJUSTMENT,
        RELEASE_FINGERS,
        RESET_ARM, // optional. can be used to move the arm down if we need to
        FINISH
    }
}