package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.commands.DriveCommand;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class AutoTrajectories {

    public static final double WAIT_TIME = 4.605;

    public static class CompAutoTrajectorySequence {
        public static final Pose2d INITIAL_POSE = new Pose2d(new Vector2d(5, -65), Math.toRadians(90));

        private final MecanumDrive DRIVE;

        public static TrajectoryActionBuilder START;
        public static TrajectoryActionBuilder TO_RUNG;
        public static TrajectoryActionBuilder TO_CORNER;
        public static TrajectoryActionBuilder TO_BLOCK_LEFT;
        public static TrajectoryActionBuilder TO_BLOCK_MIDDLE;
        public static TrajectoryActionBuilder TO_BLOCK_RIGHT;

        private final STATES[] RUN_REEL = {
                STATES.START,
                STATES.PLACE_RUNG,
                STATES.BLOCK_LEFT,
                STATES.WAIT,
                STATES.DROP_SAMPLE,
                STATES.DROP,
                STATES.WAIT,
                STATES.TO_CORNER,
                STATES.WAIT,
                STATES.TO_RUNG,
                STATES.PLACE_RUNG,
                STATES.BLOCK_MIDDLE,
                STATES.DROP_SAMPLE,
                STATES.WAIT,
                STATES.DROP,
                STATES.END
        };

        public enum STATES {
            START,
            TO_RUNG(new Pose2d(new Vector2d(5, -35), Math.toRadians(90))),
            TO_CORNER(new Pose2d(new Vector2d(64, -65), Math.toRadians(0))),
            BLOCK_LEFT(new Pose2d(new Vector2d(49, -35), Math.toRadians(90))),
            BLOCK_MIDDLE(new Pose2d(new Vector2d(60, -35), Math.toRadians(90))),
            BLOCK_RIGHT(new Pose2d(new Vector2d(70, -35), Math.toRadians(90))),
            PARK_CORNER(new Pose2d(new Vector2d(43, -65), Math.toRadians(90))),
            PARK_SUB,
            PLACE_RUNG,
            WAIT(),
            DROP_SAMPLE(new Pose2d(new Vector2d(43, -65), Math.toRadians(0))),
            DROP,
            END;

            private final Pose2d END_POSE;
            private final Pose2d START_POSE;

            STATES() {
                END_POSE = INITIAL_POSE;
                START_POSE = INITIAL_POSE;
            }

            STATES(Pose2d endPose) {
                END_POSE = endPose;
                START_POSE = INITIAL_POSE;
            }

            STATES(Pose2d startPose, Pose2d endPose) {
                START_POSE = startPose;
                END_POSE = endPose;
            }

            public Pose2d get_END_POSE() {
                return END_POSE;
            }

            public Pose2d get_START_POSE() {
                return START_POSE;
            }
        }

        public CompAutoTrajectorySequence(MecanumDrive drive) {
            DRIVE = drive;
        }



        public SequentialAction build() {
            ArrayList<Action> actions = new ArrayList<>();
            STATES previousState = STATES.START;
            for (STATES state : RUN_REEL) {
                switch (state) {
                    case START:
                        actions.add(generateStartTrajectory().build());
                        break;
                    case WAIT:
                        actions.add(new SleepAction(1.605));
                        break;
                    case PLACE_RUNG:
                        actions.add(generateToRungTrajectory(previousState).build());
                        break;
                    case TO_CORNER:
                        actions.add(generateToCorner(previousState).build());
                        break;
                    case END:
                    default:
                        return new SequentialAction(actions);
                }
                previousState = state;
            }
            return new SequentialAction(actions);
        }

        private TrajectoryActionBuilder generateStartTrajectory() {
            START = DRIVE.actionBuilder(STATES.START.get_START_POSE())
                    .lineToX(STATES.TO_RUNG.END_POSE.position.x);
            return START;
        }

        private TrajectoryActionBuilder generateToRungTrajectory(STATES previousState) {
            TO_RUNG = DRIVE.actionBuilder(previousState.get_END_POSE())
                    .splineToConstantHeading(STATES.TO_RUNG.END_POSE.position, STATES.TO_RUNG.END_POSE.heading);
            return TO_RUNG;
        }

        private TrajectoryActionBuilder generateToCorner(STATES previousState) {
            Vector2d pos = STATES.TO_CORNER.END_POSE.position;
            Rotation2d heading = STATES.TO_CORNER.END_POSE.heading;
            TO_CORNER = DRIVE.actionBuilder(previousState.get_END_POSE())
                    .splineToConstantHeading(new Vector2d(40, pos.y), heading)
                    .waitSeconds(2)
                    .splineToConstantHeading(pos, heading);

            return TO_CORNER;
        }
    }
}
