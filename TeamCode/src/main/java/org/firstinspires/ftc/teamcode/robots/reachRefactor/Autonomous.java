package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Crane;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.DriveTrain;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Function;

public class Autonomous {
    public VisionProvider visionProvider;
    private Robot robot;

    // autonomous routines
    private StateMachine blueUp, redUp, blueDown, redDown, blueUpLinear, redUpLinear, blueDownLinear, redDownLinear;

    // misc. routines
    public StateMachine backAndForth, square, turn, lengthTest;

    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    public StateMachine getStateMachine(Constants.Position startingPosition, boolean spline) {
        if(spline)
            switch(startingPosition) {
                case START_BLUE_UP:
                    return blueUp;
                case START_RED_UP:
                    return redUp;
                case START_BLUE_DOWN:
                    return blueDown;
                case START_RED_DOWN:
                    return redDown;
            }
        else
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
        return null;
    }

    private StateMachine trajectorySequenceToStateMachine(Function<TrajectorySequenceBuilder, TrajectorySequenceBuilder> function) {
        return Utils.getStateMachine(new Stage())
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                        function.apply(
                            robot.driveTrain.trajectorySequenceBuilder(
                                    robot.driveTrain.getPoseEstimate()
                            )
                        )
                        .build()
                    );
                })
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    private StateMachine trajectorySequenceToStateMachine(TrajectorySequence trajectorySequence) {
        return Utils.getStateMachine(new Stage())
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                            trajectorySequence
                    );
                })
                .build();
    }

    public void build() {
        //----------------------------------------------------------------------------------------------
        // Misc. Routines
        //----------------------------------------------------------------------------------------------
        
        TrajectorySequence backAndForthSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .forward(24)
                        .back(24)
                        .build();
        backAndForth = trajectorySequenceToStateMachine(backAndForthSequence);

        TrajectorySequence squareSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .forward(24)
                        .turn(Math.toRadians(90))
                        .build();
        square = trajectorySequenceToStateMachine(squareSequence);

        TrajectorySequence turnSequence =
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .turn(Math.toRadians(90))
                        .build();
        turn = trajectorySequenceToStateMachine(turnSequence);

        lengthTest = Utils.getStateMachine(new Stage())
                .addState(() -> {
                    robot.driveTrain.setChassisLengthMode(DriveTrain.ChassisLengthMode.SWERVE);
                    robot.driveTrain.setChassisLength(robot.driveTrain.getTargetChassisLength() + 12);
                    return robot.driveTrain.chassisLengthOnTarget();
                })
                .addSingleState(() -> {
                    robot.driveTrain.followTrajectorySequenceAsync(
                            robot.driveTrain.trajectorySequenceBuilder(
                                    robot.driveTrain.getPoseEstimate()
                            )
                                    .forward(12)
                                    .build()
                    );
                })
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .addState(() -> {
                    robot.driveTrain.setChassisLengthMode(DriveTrain.ChassisLengthMode.DIFF);
                    robot.driveTrain.setChassisLength(robot.driveTrain.getTargetChassisLength() - 12);
                    return robot.driveTrain.chassisLengthOnTarget();
                })
                .build();

        //----------------------------------------------------------------------------------------------
        // Spline Routines
        //----------------------------------------------------------------------------------------------

        blueUp = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                    builder
                    .splineTo(new Vector2d(0, 43), Math.toRadians(235))
                ))
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                    builder
                    .turn(Math.toRadians(-75))
                    .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                ))
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                        builder
                        .back(120)
                ))
                .build();
        redUp = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                    builder
                    .splineTo(new Vector2d(0, -43), Math.toRadians(120))
                ))
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                    builder
                    .turn(Math.toRadians(75))
                    .splineTo(new Vector2d(-60, -60), Math.toRadians(180))
                ))
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                        builder
                        .back(120)
                    )
                )
                .build();

        blueDown = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                        builder
                        .splineTo(new Vector2d(-24, 40), Math.toRadians(315))
                ))
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(builder ->
                        builder
                        .turn(Math.toRadians(-135))
                        .splineTo(new Vector2d(-60, 60), Math.toRadians(180))
                ))
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();
        redDown = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .splineTo(new Vector2d(-24, -40), Math.toRadians(45))
                        )
                )
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(135))
                                .splineTo(new Vector2d(-60, -60), Math.toRadians(180))
                        )
                )
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();

        //----------------------------------------------------------------------------------------------
        // Linear Routines
        //----------------------------------------------------------------------------------------------

        blueUpLinear = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(-35))
                                .forward(20)
                        )
                )
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(-70))
                                .forward(58)
                                .turn(Math.toRadians(15))
                                .forward(5)
                        )
                )
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();
        redUpLinear = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(35))
                                .forward(20)
                        )
                )
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(70))
                                .forward(58)
                                .turn(Math.toRadians(-15))
                                .forward(5)
                        )
                )
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();

        blueDownLinear = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(35))
                                .forward(23)
                        )
                )
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(-150))
                                .forward(40)
                                .turn(Math.toRadians(25))
                                .forward(2)
                        )
                )
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();
        redDownLinear = Utils.getStateMachine(new Stage())
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(-35))
                                .forward(23)
                        )
                )
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> { robot.crane.articulate(Crane.Articulation.LOWEST_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.MIDDLE_TIER); return true; },
                        () -> { robot.crane.articulate(Crane.Articulation.HIGH_TIER); return true; }
                )
                .addState(() -> robot.crane.getArticulation().equals(Crane.Articulation.MANUAL))
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .turn(Math.toRadians(150))
                                .forward(40)
                                .turn(Math.toRadians(-25))
                                .forward(2)
                        )
                )
                .addTimedState(
                        4,
                        () -> robot.driveTrain.toggleDuckSpinner(1),
                        () -> robot.driveTrain.toggleDuckSpinner(1)
                )
                .addNestedStateMachine(trajectorySequenceToStateMachine(
                        builder -> builder
                                .back(120)
                        )
                )
                .build();
    }

    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }
}
