package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config

@Autonomous(name = "Blue_Park")
public class BlueAutoPark extends PowerPlay_AprilTagDetection {
    //Increasing makes the bot move less; don't ask
    public static double forwardDistance=39;
    public static double sideDistance=26;
    public static double angle=-270;
    public void initilize(){
        super.runOpMode();
    }


    @Override
    public void runOpMode() {
        initilize();
        Pose2d startPose = new Pose2d(-35, 62, Math.toRadians(-270));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        if (aprilTag_ID == 3) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .strafeRight(sideDistance)
                    .build();

            bot.followTrajectorySequence(sussyBaka);
            telemetry.addData("Chris Pratt", "Is Currently In The Mushroom Kingdom");
        } else if (aprilTag_ID == 2) {
            TrajectorySequence JuicyJay = bot.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .build();

            bot.followTrajectorySequence(JuicyJay);
            telemetry.addData("Walter White", "Currently Has No Pants");
        } else {
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .strafeLeft(sideDistance)
                    .build();

            bot.followTrajectorySequence(jacobIsCute);
            telemetry.addData("Bohan and Abhilash", " = Very Cute");
        }
    }
}
