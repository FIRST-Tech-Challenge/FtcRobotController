package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Variables.VisionProcessors;
import org.firstinspires.ftc.teamcode.Variables.Detection;

import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.util.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "BackBlueAuto", group = "Linear OpMode")
public class BackBlueAuto extends MeepMeepBoilerplate{
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo passiveServo = hardwareMap.get(Servo.class, "passiveServo");
        initVision(VisionProcessors.TFOD);
        Detection detection = Detection.UNKNOWN;
        while (opModeInInit()) {
            detection = getDetectionsSingleTFOD();
            telemetry.addData("Detection", detection);
            telemetry.update();
        }
        switch (detection) {
            case LEFT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(26.0)
                            .turn(Math.toRadians(90))
                            .forward(10)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.5))
                            .waitSeconds(.25)
                            .back(10)
                            .strafeRight(25)
                            .forward(90)
                            .build()
            );
            case CENTER -> { drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(33)
                            .addDisplacementMarker(() -> passiveServo.setPosition(0.5))
                            .build());
            }
            case RIGHT -> drive.followTrajectorySequence(
                    drive.trajectorySequenceBuilder(getCurrentPosition(drive))
                            .forward(28.0)
                            .turn(Math.toRadians(-90))
                            .forward(6)
                            .addTemporalMarker(() -> passiveServo.setPosition(0.5))
                            .waitSeconds(.25)
                            .back(8)
                            .strafeLeft(12)
                            .back(85)
                            .build()
            );
            default -> {
                telemetry.addLine("Warning: Cup not detected");
                telemetry.update();
                sleep(3000);
            }
        }



//        drive.followTrajectorySequence(mergeSequences(sequences));
    }
}
