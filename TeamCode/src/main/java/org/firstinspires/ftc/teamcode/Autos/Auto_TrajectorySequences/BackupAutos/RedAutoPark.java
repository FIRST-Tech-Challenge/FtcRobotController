package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Autonomous(name = "RedAutoPark")
@Config
public class RedAutoPark extends PowerPlay_AprilTagDetection {

    private Claw clawControl;

    public static double forwardDistance=86.7;
    public static double sideDistance=26;
    public static double angle=270;

    public void initialize()
    {
        clawControl = new Claw(hardwareMap);
    }

    public void scan(){
        super.runOpMode();

    }
    @Override
    public void runOpMode() {
        initialize();
        scan();
        Pose2d startPose = new Pose2d(-35, 62, Math.toRadians(angle));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);
        //clawControl.toggleAutoOpenClose();
        if (aprilTag_ID == 3) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(startPose)
                    //.waitSeconds(2)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .strafeLeft(sideDistance)
                    .build();

            bot.followTrajectorySequence(sussyBaka);
            telemetry.addData("Chris Pratt", "Is Currently In The Mushroom Kingdom");
        } else if (aprilTag_ID == 2) {
            TrajectorySequence JuicyJay = bot.trajectorySequenceBuilder(startPose)
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .build();

            bot.followTrajectorySequence(JuicyJay);
            telemetry.addData("Walter White", "Currently Has No Pants");
        } else {
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(startPose)
                    .waitSeconds(0.5)
                    .lineToLinearHeading(new Pose2d(-35,forwardDistance,Math.toRadians(angle)))
                    .strafeRight(sideDistance)
                    .build();

            bot.followTrajectorySequence(jacobIsCute);
            telemetry.addData("Bohan and Abhilash", " = Very Cute");
        }
    }
}