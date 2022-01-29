package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.Position;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProvider;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.vision.VisionProviders;
import org.firstinspires.ftc.teamcode.statemachine.Stage;
import org.firstinspires.ftc.teamcode.statemachine.StateMachine;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class Autonomous {
    public VisionProvider visionProvider;
    private Robot robot;

    // Autonomous trajectories
    TrajectorySequence sequence;

    // Autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed;

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue;

    public Autonomous(Robot robot) {
        this.robot = robot;

        // Autonomous trajectories
        sequence = robot.driveTrain.trajectorySequenceBuilder(new Pose2d(0, 0))
                .forward(24)
                .turn(Math.toRadians(90))
                .splineTo(new Vector2d(48, 48), Math.toRadians(90))
                .build();;

        // Autonomous articulations
        autonomousRed = getStateMachine(autonomousRedStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(sequence))
                .addState(() -> !robot.driveTrain.trajectorySequenceRunner.isBusy())
                .build();

        autonomousBlue = getStateMachine(autonomousBlueStage)
                .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                        () -> true,
                        () -> true,
                        () -> true
                )
                .addSingleState(() -> robot.driveTrain.followTrajectorySequenceAsync(sequence))
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
