package org.firstinspires.ftc.teamcode.robots.reachRefactor.opmodes;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Autonomous {
    public VisionProvider visionProvider;
    private Robot robot;

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

    private StateMachine trajectorySequenceToArticulation(Stage stage, TrajectorySequence trajectorySequence) {
        return getStateMachine(stage)
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(trajectorySequence))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();
    }

    public void build() {
        // trajectory articulations
        backAndForth = trajectorySequenceToArticulation(
                backAndForthStage,
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                    .forward(24)
                    .back(24)
                    .build()
        );
        square = trajectorySequenceToArticulation(
                squareStage,
                robot.driveTrain.trajectorySequenceBuilder(robot.driveTrain.getPoseEstimate())
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .forward(24)
                    .turn(Math.toRadians(90))
                    .build()
        );

        // autonomous articulations
        autonomousRed = getStateMachine(autonomousRedStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
                .build();

        autonomousBlue = getStateMachine(autonomousBlueStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
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
