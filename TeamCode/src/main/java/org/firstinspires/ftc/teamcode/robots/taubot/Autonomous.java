package org.firstinspires.ftc.teamcode.robots.taubot;

import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.MIN_CHASSIS_LENGTH;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.Position;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.wrapAngle;
import static org.firstinspires.ftc.teamcode.util.utilMethods.futureTime;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.taubot.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.taubot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Utils;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.Target;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.provider.DPRGCanDetectorProvider;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Config (value = "AA_PP_Auton")
public class Autonomous implements TelemetryProvider {
    public VisionProvider visionProvider;
    private Robot robot;

    @Override
    public Map<String, Object> getTelemetry(boolean debug) {
        Map<String, Object> telemetryMap = new LinkedHashMap<>();

        //telemetryMap.put("Current Articulation", mode );
        telemetryMap.put("Can Stage", sixCanStage);
        return telemetryMap;
    }

    @Override
    public String getTelemetryName() {
        return "Autonomous";
    }

    enum Mode {
        LINEAR, SPLINE, NO_RR, SIMPLE;
    }

    // autonomous routines
    private StateMachine left, right, blueDown, redDown,
            blueUpLinear, redUpLinear, blueDownLinear, redDownLinear,
            leftNoRR, rightNoRR, blueDownNoRR, redDownNoRR,
            blueUpSimple, redUpSimple, blueDownSimple, redDownSimple;
    // misc. routines
    public StateMachine backAndForth, square, turn, lengthTest, diagonalTest, squareNoRR, Auton;

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    public StateMachine getStateMachine(Position startingPosition, Mode mode) {
        switch(mode) {
            case LINEAR:
                switch(startingPosition) {
                    case START_LEFT:
                        return left;
                    case START_RIGHT:
                        return right;
                }
                break;
            case SPLINE:
/*
                switch(startingPosition) {
                    case START_BLUE_UP:
                        return blueUpLinear;
                    case START_RED_UP:
                        return redUpLinear;
                    case START_BLUE_DOWN:
                        return blueDownLinear;
                    case START_RED_DOWN:
                        return redDownLinear;
                }
*/
                break;
            case NO_RR:
                switch(startingPosition) {
                    case START_LEFT:
                        return leftNoRR;
                    case START_RIGHT:
                        return rightNoRR;
                }
                break;
            case SIMPLE:
                /*
                switch(startingPosition) {
                    case START_BLUE_UP:
                        return blueUpSimple;
                    case START_RED_UP:
                        return redUpSimple;
                    case START_BLUE_DOWN:
                        return blueDownSimple;
                    case START_RED_DOWN:
                        return redDownSimple;
                }
                */

                break;
        }

        return null;
    }

    private StateMachine trajectorySequenceToStateMachine(TrajectorySequence trajectorySequence) {
        return Utils.getStateMachine(new Stage())
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                            trajectorySequence
                    );
                })
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    public void build(Position startingPosition) {
        //----------------------------------------------------------------------------------------------
        // Misc. Routines
        //----------------------------------------------------------------------------------------------
        
        TrajectorySequence backAndForthSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(24)
                        .forward(24)
                        .build();
        backAndForth = trajectorySequenceToStateMachine(backAndForthSequence);

        TrajectorySequence squareSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .back(12)
                        .turn(Math.toRadians(-90))
                        .build();
        square = trajectorySequenceToStateMachine(squareSequence);

        squareNoRR = Utils.getStateMachine(new Stage())
                .addState(() -> robot.driveTrain.driveUntilDegrees(24, 0,20))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.turnUntilDegrees(-90))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.driveUntilDegrees(24, -90,20))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.turnUntilDegrees(-180))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.driveUntilDegrees(24, -180,20))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.turnUntilDegrees(-270))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.driveUntilDegrees(24, -270, 20))
                .addTimedState(1f, () -> {}, () -> {})
                .addState(() -> robot.driveTrain.turnUntilDegrees(0))
                .addTimedState(1f, () -> {}, () -> {})
                .build();

        TrajectorySequence turnSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .build();
        turn = trajectorySequenceToStateMachine(turnSequence);



        //----------------------------------------------------------------------------------------------
        // Spline Routines
        //----------------------------------------------------------------------------------------------

        switch(startingPosition) {
            case START_LEFT:
            /* sample code from Reach
                left = Utils.getStateMachine(new Stage())

                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addTimedState(3f, () -> robot.crane.turret.setTargetHeading(90), () -> {})
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> robot.crane.articulate(Crane.Articulation.HOME))
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        blueUp2
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();
*/

                leftNoRR = Utils.getStateMachine(new Stage())
                        .addTimedState(1, () -> {
                        }, () -> {
                        }) //wait

                        //drive until we approach conestack
                        .addState(() -> robot.driveTrain.driveUntil(-20, 20))
                        //turn to warehouse
                        .addState(() -> robot.driveTrain.turnUntilDegrees(-90))
                        //enter warehouse
                        .addState(() -> robot.driveTrain.driveUntilDegrees(16, wrapAngle(-45), 20))
                        //.addTimedState(3,()->{}, ()->{}) //wait
                        .addState(() -> robot.driveTrain.driveUntilDegrees(26, wrapAngle(45), 20))

                        .build();
                break;
            case START_RIGHT:
                rightNoRR = Utils.getStateMachine(new Stage())

                        //start moving the arm so subsequent movements aren't as large
                        .addTimedState(1, () -> {
                        }, () -> {
                        }) //wait so arm movement doesn't build on kinetic energy of chassis

//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.HIGH_TIER))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)

                        .addTimedState(1, () -> {
                        }, () -> {
                        }) //wait

                        .addState(() -> robot.driveTrain.driveUntil(1.5 * 23.5, 20))
                        //turn to warehouse
                        .addState(() -> robot.driveTrain.turnUntilDegrees(45))
                        //enter warehouse
                        .addState(() -> robot.driveTrain.driveUntilDegrees(16, wrapAngle(45), 20))
                        //.addTimedState(3,()->{}, ()->{}) //wait
                        .addState(() -> robot.driveTrain.driveUntilDegrees(26, wrapAngle(-45), 20))

                        .build();
/* samples from Reach
                TrajectorySequence redUp1Simple = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(20)
                        .build();
                TrajectorySequence redUp2Simple = robot.driveTrain.trajectorySequenceBuilder(redUp1Simple.end())
                        .turn(Math.toRadians(-90))
                        .back(70)
                        .build();
                redUpSimple = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    redUp1Simple
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        redUp2Simple
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();
                break;


            case START_BLUE_DOWN:
                TrajectorySequence blueDown1 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(29.67)
                        .build();
                TrajectorySequence blueDown2 = robot.driveTrain.trajectorySequenceBuilder(blueDown1.end())
                        .splineTo(new Vector2d(-68, 68), Math.toRadians(135))
                        .build();
                TrajectorySequence blueDown3 = robot.driveTrain.trajectorySequenceBuilder(blueDown2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, 0), Math.toRadians(0))
                        .splineTo(new Vector2d(12, 12), Math.toRadians(45))
                        .back(50)
                        .turn(Math.toRadians(180))
                        .build();
                blueDown = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    blueDown1
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() -> robot.crane.turret.setTargetHeading(-90))
                        .addState(() -> robot.turret.isTurretNearTarget())
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> robot.crane.articulate(Crane.Articulation.HOME))
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                    blueDown2
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addTimedState(3f,
                                () -> robot.driveTrain.toggleDuckSpinner(Alliance.BLUE.getMod()),
                                () -> robot.driveTrain.toggleDuckSpinner(Alliance.BLUE.getMod())
                        )
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                    blueDown3
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();

                TrajectorySequence blueDownLinear1 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(31.67)
                        .build();
                TrajectorySequence blueDownLinear2 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(38.33)
                        .turn(-Math.toRadians(90))
                        .forward(48)
                        .turn(Math.toRadians(90))
                        .forward(36)
                        .turn(-Math.toRadians(90))
                        .forward(36)
                        .build();
                blueDownLinear = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    blueDownLinear1
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_HIGH_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addTimedState(3f, () -> robot.crane.turret.setTargetHeading(-90), () -> {})
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> robot.crane.articulate(Crane.Articulation.HOME))
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        blueDownLinear2
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .build();

                blueDownNoRR = Utils.getStateMachine(new Stage())
                        .build();

                TrajectorySequence blueDown1Simple = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(20)
                        .build();
                TrajectorySequence blueDown2Simple = robot.driveTrain.trajectorySequenceBuilder(blueDown1Simple.end())
                        .turn(-Math.toRadians(90))
                        .forward(98)
                        .build();
                blueDownSimple = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    blueDown1Simple
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        blueDown2Simple
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();
                break;
            case START_RED_DOWN:
                TrajectorySequence redDown1 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(29.67)
                        .build();
                TrajectorySequence redDown2 = robot.driveTrain.trajectorySequenceBuilder(redDown1.end())
                        .splineTo(new Vector2d(-68, -68), Math.toRadians(215))
                        .build();
                TrajectorySequence redDown3 = robot.driveTrain.trajectorySequenceBuilder(redDown2.end())
                        .setReversed(true)
                        .splineTo(new Vector2d(-24, 0), Math.toRadians(0))
                        .splineTo(new Vector2d(12, -12), Math.toRadians(315))
                        .back(50)
                        .turn(Math.toRadians(180))
                        .build();
                redDown = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    redDown1
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() -> robot.crane.turret.setTargetHeading(90))
                        .addState(() -> robot.turret.isTurretNearTarget())
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> robot.crane.articulate(Crane.Articulation.HOME))
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        redDown2
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addTimedState(3f,
                                () -> robot.driveTrain.toggleDuckSpinner(Alliance.RED.getMod()),
                                () -> robot.driveTrain.toggleDuckSpinner(Alliance.RED.getMod())
                        )
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        redDown3
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();

                TrajectorySequence redDownLinear1 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(31.67)
                        .build();
                TrajectorySequence redDownLinear2 = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(38.33)
                        .turn(Math.toRadians(90))
                        .forward(48)
                        .turn(-Math.toRadians(90))
                        .forward(36)
                        .turn(Math.toRadians(90))
                        .forward(36)
                        .build();

                redDownLinear = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    redDownLinear1
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_HIGH_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addTimedState(3f, () -> robot.crane.turret.setTargetHeading(90), () -> {})
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> robot.crane.articulate(Crane.Articulation.HOME))
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        redDownLinear2
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .build();

                redDownNoRR = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {robot.driveTrain.setChassisLengthMode(DriveTrain.ChassisLengthMode.BOTH);})
                        .addSingleState(() -> {
                            robot.crane.setToHomeEnabled(false);
                            //robot.driveTrain.setUseAutonChassisLengthPID(true);
                        })
                        //hack pre-align swerve steer by driving into wall
                        .addState(() -> robot.driveTrain.driveUntil(1,robot.driveTrain.getPoseEstimate().getHeading() ,.1))
                        .addTimedState(1,()->{}, ()->{}) //wait

                        //extend swerve
                        .addSingleState(() -> robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH +9))
                        //start moving the arm so subsequent movenments aren't as large
                        .addTimedState(1,()->{}, ()->{}) //wait so arm movement doesn't build on kinetic energy of chassis

                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.HIGH_TIER))
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)

                        .addTimedState(1,()->{}, ()->{}) //wait

                        //preload dump section
                        .addMineralState(
                                () -> visionProvider.getMostFrequentPosition().getIndex(),
                                () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                                () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                        )
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)

                        //turn turret to shared hub, then dump and return
                        .addTimedState(2f, () -> robot.crane.turret.setTargetHeading(-45), () -> {})
                        .addTimedState(1.5f, () -> robot.crane.dump(), () -> {})
                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        .addTimedState(1f, () -> robot.crane.turret.setTargetHeading(0), () -> {})
                        //.addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
                        //preload dump section end

//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_PREP))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)

                        //retract swerve
                        .addSingleState(() -> {robot.driveTrain.setChassisLengthMode(DriveTrain.ChassisLengthMode.SWERVE);})
                        .addSingleState(() -> robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))

                        //FFUTSE Retrieval
//                        .addMineralState(
//                                () -> visionProvider.getMostFrequentPosition().getIndex(),
//                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_LEFT); return true; },
//                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_MIDDLE); return true; },
//                                () -> { robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_RIGHT); return true; }
//                        )
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_UP))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.AUTON_FFUTSE_HOME))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.STOW_FFUTSE))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)
//                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.RELEASE_FFUTSE))
//                        .addState(() -> robot.crane.getArticulation() == Crane.Articulation.MANUAL)


                        .addSingleState(() -> {
                            robot.crane.setToHomeEnabled(true);
                            //robot.driveTrain.setUseAutonChassisLengthPID(true);
                        })
                        //end pickup FFUTSE sequence

                        //turn toward warehouse
                        .addSingleState(() -> robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH))
                        .addState(() -> robot.driveTrain.driveUntil(-20,20))
                        //turn to warehouse
                        .addState(() -> robot.driveTrain.turnUntilDegrees(-45))
                        //enter warehouse
                        .addState(() -> robot.driveTrain.driveUntilDegrees(16, wrapAngle(-45),20))
                        //.addTimedState(3,()->{}, ()->{}) //wait
                        .addState(() -> robot.driveTrain.driveUntilDegrees(26, wrapAngle(45),20))

                        .build();

                TrajectorySequence redDown1Simple = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(20)
                        .build();
                TrajectorySequence redDown2Simple = robot.driveTrain.trajectorySequenceBuilder(redDown1Simple.end())
                        .turn(Math.toRadians(90))
                        .forward(98)
                        .build();
                redDownSimple = Utils.getStateMachine(new Stage())
                        .addSingleState(() -> {
                            robot.driveTrain.followTrajectorySequenceAsync(
                                    redDown1Simple
                            );
                        })
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() ->
                                robot.driveTrain.followTrajectorySequenceAsync(
                                        redDown2Simple
                                )
                        )
                        .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                        .addSingleState(() -> robot.crane.articulate(Crane.Articulation.TRANSFER))
                        .build();
                break;
        }
       */
        }
        TrajectorySequence diagonalTestTrajectory =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .back(72)
                        .turn(Math.toRadians(180))
                        .back(48)
                        .turn(Math.toRadians(-45))
                        .back(100)
                        .build();
        diagonalTest = trajectorySequenceToStateMachine(diagonalTestTrajectory);
    }


    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }
    int canScannerStage = 0;

    List<Target> uniqueTargets = new ArrayList<>();

    public boolean CanScanner(){
        switch (canScannerStage) {
            case 0: //initialize stuff
                uniqueTargets.clear(); //starting a new scan
                //todo make sure we've setup the pipeline?
                canScannerStage++;
                break;
            case 1:
            if (robot.driveTrain.turnUntilDegrees(-90)) {
                return true; //done rotating/scanning
            }
            else {


            }
        }

        return false;
    }

    public int sixCanStage = 0;
    long sixCanTimer;
    public Target scoreFrom = new Target();


    public boolean SixCan(){
        DPRGCanDetectorProvider sixVision;
        List<Target> uniqueCans = null;
        List<Target> frameCans = null;
        sixVision = (DPRGCanDetectorProvider) visionProvider;
        Target currentTarget;
        Pose2d robotLocation;

        switch (sixCanStage) {
            case 0: //initialize stuff
                //todo, these calls to setHeading and setPoseEstimate should be combined into a utility method to keep them in sync
                //todo should never call setPoseEstimate if the heading is nonzero without also setting the Heading
                robot.driveTrain.setHeading(Position.START_SIXCAN.getPose().getHeading());
                robot.driveTrain.setPoseEstimate(Position.START_SIXCAN.getPose());

                robot.driveTrain.articulate(DriveTrain.Articulation.unlock);
                robot.driveTrain.enableChassisLength();
                robot.driveTrain.maxTuck();
                uniqueCans = sixVision.getUniqueCans();
                uniqueCans.clear(); //starting a fresh set of unique cans
                //todo make sure we've setup the pipeline?
                sixCanStage++;
                break;
            case 1:
                //start facing right and turn left until pointing down the field
                if (robot.driveTrain.turnUntilDegrees(0)) {
                    sixCanStage++; //progress to next stage
                }
                break;
            case 2: //find starting can
                //find the first can that is close to the centerline
                // can where abs of the y coordinate < 12 and x is smallest (closest to start)
                // if that doesn't find a target, get keep increasing the zone until a can is found
                sixCanStage++;
                break;
            case 3: //select target and approach it

                frameCans = sixVision.getFrameDetections(); //get updated list
                robotLocation=robot.driveTrain.poseEstimate;
                currentTarget = ((DPRGCanDetectorProvider) visionProvider).GetNearest(frameCans,robotLocation);

                if (currentTarget == null){ //don't see a target - let's turn in place
                    robot.driveTrain.turnUntilDegrees(Math.toDegrees(robotLocation.getHeading()) + 10);
                    //todo how do we terminate if there are no cans and we've scanned more than a full turn?
                }
                else { //let's go get the can
                    //during the turn portion it's possible a closer can comes into view and that one becomes the new target
                    if (ApproachTarget(currentTarget, 30)) {
                        sixCanTimer = futureTime(1); //some settling time
                        sixCanStage++;
                    }
                }
                break;
            case 4: //fine tune the distance to the can
                if (System.nanoTime()>sixCanTimer)
                    //if ()
                    sixCanStage++;
                break;

            case 5: //deploy  the underarm.
                if (robot.underarm.SixCanPickupPrep()) {
                    sixCanTimer = futureTime(1);
                    sixCanStage++;
                }
                break;
            case 6: //extend to engage gripper
                robot.driveTrain.setChassisLength(MIN_CHASSIS_LENGTH + 10);
                if (System.nanoTime()>sixCanTimer) {sixCanStage++;}
                break;

            case 7: //extend chariot to the fine-tuned distance to trigger the gripper
                sixCanStage++;
                break;
            case 8: //pick up the can
                if (robot.underarm.SixCanPickup()) {
                    robot.driveTrain.maxTuck();
                    sixCanStage++;
                }
                break;
            case 9: // drive to scoring location
                sixCanStage++;
                break;
            case 10: // score can
                sixCanStage++;
                break;
                // remove current target from uniqueCans
                // get nearest can, repeat
            case 11: // starting orientation
                //resume facing right and turn left until pointing down the field
                if (robot.driveTrain.turnUntilDegrees(0)) {
                    sixCanStage++; //progress to next stage
                }
                break;
            default:
                sixCanStage = 0;
                return true;
        }

        return false;
    }
    int approachTargetStage = 0;

    public boolean ApproachTarget(Target target, double stoppingDistanceInches){
        if (target == null) return false; //this helps if ApproachTarget was called before a target was ready - could be a problem because it becomes an endless loop if a Target is never supplied
        Pose2d robotLocation=robot.driveTrain.poseEstimate;
        Vector2d robotVec = new Vector2d(robot.driveTrain.poseEstimate.getX(), robot.driveTrain.poseEstimate.getY());
        double approachHeadingDegrees = Math.toDegrees( robotLocation.getHeading())+target.getCameraHeading();
        double approachDistance = target.Distance(new Vector2d(robotLocation.getX(),robotLocation.getY()))-stoppingDistanceInches;
        //Approach Target has two stages - first turn in place toward the target, second drive toward the target but stop short
        switch (approachTargetStage) {
            case 0: //turn until facing the target
                    if (robot.driveTrain.turnUntilDegrees(approachHeadingDegrees)) {
                        approachTargetStage++;
                    }
                break;
            case 1:  //drive until we get to the right distance to deploy the gripper
                    //this can include backing up if we are too close to the target
                    //so it's best to approach a target from the center of the field
                    if (robot.driveTrain.driveUntilDegrees(approachDistance, approachHeadingDegrees, 15))
                        approachTargetStage++;
                break;
            case 2:
                approachTargetStage = 0;
                return true;
            default:
                approachTargetStage = 0;
        }
        return false;
    }

    int approachLocationStage = 0;
    public boolean ApproachLocation(Pose2d targetLocation, double stoppingDistanceInches){
        if (targetLocation == null) return false; //this helps if ApproachLocation was called before a location was ready - could be a problem because it becomes an endless loop if a Target is never supplied
        Pose2d robotLocation=robot.driveTrain.poseEstimate;
        Vector2d robotVec = new Vector2d(robot.driveTrain.poseEstimate.getX(), robot.driveTrain.poseEstimate.getY());
        Vector2d targetVec = new Vector2d(targetLocation.getX(), targetLocation.getY());
        //double approachHeadingDegrees = Math.toDegrees( robotLocation.getHeading())+targetLocation.getCameraHeading();
        double approachHeadingDegrees = robotVec.angleBetween(targetVec);
        //double approachDistance = targetLocation.Distance(new Vector2d(robotLocation.getX(),robotLocation.getY()))-stoppingDistanceInches;
        double approachDistance = robotVec.distTo(targetVec) - stoppingDistanceInches;
                //ApproachLocation has two stages - first turn in place toward the location, second drive toward the location but stop short
        switch (approachLocationStage) {
            case 0: //turn until facing the location
                if (robot.driveTrain.turnUntilDegrees(approachHeadingDegrees)) {
                    approachLocationStage++;
                }
                break;
            case 1:  //drive until we get to the right distance to deploy the gripper
                //this can include backing up if we are too close to the location
                //so it's best to approach a location from the center of the field
                if (robot.driveTrain.driveUntilDegrees(approachDistance, approachHeadingDegrees, 15))
                    approachLocationStage++;
                break;
            case 2:
                approachLocationStage = 0;
                return true;
            default:
                approachLocationStage = 0;
        }
        return false;
    }
}
