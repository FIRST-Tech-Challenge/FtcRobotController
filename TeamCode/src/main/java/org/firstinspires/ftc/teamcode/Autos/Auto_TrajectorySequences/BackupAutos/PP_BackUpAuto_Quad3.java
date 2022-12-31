package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class PP_BackUpAuto_Quad3 extends PowerPlay_AprilTagDetection {

    @Override
    public void runOpMode() {
        super.runOpMode();
        Pose2d startPose = new Pose2d(-35, 61.8, Math.toRadians(270));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);

        if(aprilTag_ID == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(-12.2, 15.5, Math.toRadians(-109)))
                    .build();
            bot.followTrajectorySequenceAsync(sussyBaka);
            telemetry.addData("Chris Pratt","Is Currently In The Mushroom Kingdom");
        }else if(aprilTag_ID ==2) {
            telemetry.addData("Walter White","Curently Has No Pants");
            // bot doesn't need to move if it is in the 2nd tag zone since that is where we deposit anyway
        }else{
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(startPose)
                    .lineTo(new Vector2d(-58.2,24.6))
                    .build();

            bot.followTrajectorySequenceAsync(jacobIsCute);
            telemetry.addData("Bohan and Abhilash"," = Very Cute");
        }
    }
}
