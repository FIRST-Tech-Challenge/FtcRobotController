package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Autonomous {
    public VisionProvider visionProvider;
    private Robot robot;

    // trajectories
    TrajectorySequence backAndForthTrajectory, squareTrajectory;

    // autonomous trajectories
    // TODO: create autonomous trajectories using MeepMeep

    // autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed;

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue;

    // trajectory articulations
    private Stage backAndForthStage = new Stage();
    public StateMachine backAndForth;

    private Stage squareStage = new Stage();
    public StateMachine square;


    public Autonomous(Robot robot) {
        this.robot = robot;
    }

    public void build() {
        // trajectories
        backAndForthTrajectory = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                .forward(24)
                .back(24)
                .build();

        squareTrajectory = robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .turn(Math.toRadians(90))
                .forward(24)
                .turn(Math.toRadians(90))
                .build();

        // trajectory articulations
        backAndForth = getStateMachine(backAndForthStage)
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(backAndForthTrajectory))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();

        square = getStateMachine(squareStage)
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(squareTrajectory))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();

        // autonomous articulations
        autonomousRed = getStateMachine(autonomousRedStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(squareTrajectory))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();

        autonomousBlue = getStateMachine(autonomousBlueStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(squareTrajectory))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    private StateMachine.Builder getStateMachine(Stage stage) {
        return StateMachine.builder()
                .stateSwitchAction(() -> {})
                .stateEndAction(() -> {})
                .stage(stage);
    }

    public void createVisionProvider(int visionProviderIndex) {
        try {
            visionProvider = VisionProviders.VISION_PROVIDERS[visionProviderIndex].newInstance();
        } catch(IllegalAccessException | InstantiationException e) {
            throw new RuntimeException("Error while instantiating vision provider");
        }
    }
}
