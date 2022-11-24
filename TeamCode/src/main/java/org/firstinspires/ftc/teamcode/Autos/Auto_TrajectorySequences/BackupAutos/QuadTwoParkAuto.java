package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "QuadTwoParkAuto")
public class QuadTwoParkAuto extends PowerPlay_AprilTagDetection {
    public void initilize(){
        super.runOpMode();
    }
    @Override
    public void runOpMode() {
        initilize();
        Pose2d startPose = new Pose2d(35, 62, Math.toRadians(270));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        if (tagUse == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(startPose)
                    .back(19.5)
                    .splineTo(new Vector2d(11.6,89.5),Math.toRadians(-80))
                    .build();

            bot.followTrajectorySequence(sussyBaka);
            telemetry.addData("Chris Pratt", "Is Currently In The Mushroom Kingdom");
        } else if (tagUse == 2) {
            TrajectorySequence JuicyJay = bot.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(35,95,Math.toRadians(0)))
                    .build();

            bot.followTrajectorySequence(JuicyJay);
            telemetry.addData("Walter White", "Currently Has No Pants");
        } else {
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(startPose)
                    .back(20)
                    .splineTo(new Vector2d(58.4,34.5),Math.toRadians(-40))
                    .build();

            bot.followTrajectorySequenceAsync(jacobIsCute);
            telemetry.addData("Bohan and Abhilash", " = Very Cute");
        }
    }
}