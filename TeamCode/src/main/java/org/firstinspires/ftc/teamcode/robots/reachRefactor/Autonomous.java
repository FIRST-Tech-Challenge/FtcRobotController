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

    public Autonomous(Robot robot) {
        this.robot = robot;
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

    // Autonomous trajectories
    TrajectorySequence sequence =  robot.driveTrain.trajectorySequenceBuilder(new Pose2d(0, 0))
            .lineTo(new Vector2d(10, 10))
            .turn(Math.toRadians(90))
            .build();

    // Autonomous articulations
    private Stage autonomousRedStage = new Stage();
    public StateMachine autonomousRed = getStateMachine(autonomousRedStage)
            .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                    () -> true,
                    () -> true,
                    () -> true
            )
            .addTrajectoryState(robot.driveTrain.trajectorySequenceRunner, sequence)
            .build();

    private Stage autonomousBlueStage = new Stage();
    public StateMachine autonomousBlue = getStateMachine(autonomousBlueStage)
            .addMineralState(() -> visionProvider.getMostFrequentPosition().getIndex(),
                    () -> true,
                    () -> true,
                    () -> true
            )
            .build();
}
