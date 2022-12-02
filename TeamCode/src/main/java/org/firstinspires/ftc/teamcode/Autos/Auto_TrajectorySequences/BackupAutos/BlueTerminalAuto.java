package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetection;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueAutoPark")
public class BlueTerminalAuto extends PowerPlay_AprilTagDetectionDeposit {
    private Arm armControl;
    private Slide slideControl;
    private Claw clawControl;
    private boolean isAuto = true;

    public void initialize(){
        super.runOpMode();
    }

    @Override
    public void runOpMode() {
        int currentTag = 0;
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawControl = new Claw(hardwareMap);

        Pose2d startPose = new Pose2d(-35, 62, Math.toRadians(0));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        // TRAJECTORY SEQUENCES \\
        TrajectorySequence rotate = bot.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(new Pose2d(-35,50,Math.toRadians(180))) // rotates the bot
            .waitSeconds(2) // waits to scan AprilTag
            .build();

        TrajectorySequence forward_extake = bot.trajectorySequenceBuilder(rotate.end())
            .lineToLinearHeading(new Pose2d(-35, 13, Math.toRadians(-39.75)))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                slideControl.setHighJunction();
                armControl.setExtake();
                clawControl.toggleWristRotate();
                })
            .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                clawControl.toggleOpenClose();
                armControl.setIntake();
                slideControl.setIntakeOrGround();
                })
            .waitSeconds(1.5) // giving the claw time to open, otherwise it would instantly open immediately following wristRotate()
            .build();

        TrajectorySequence backToPosition = bot.trajectorySequenceBuilder(forward_extake.end())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> clawControl.toggleOpenClose())
                .lineToLinearHeading(new Pose2d(-35,35,Math.toRadians(-270)))
                .build();

        //  FOLLOW TRAJECTORY METHOD CALLS  \\
        waitForStart();
        bot.followTrajectorySequenceAsync(rotate);
        initialize();
        currentTag = tagUse;
        bot.followTrajectorySequenceAsync(forward_extake);
        bot.followTrajectorySequenceAsync(backToPosition);

        //  APRIL TAGS \\
        if (currentTag == 1) {
            TrajectorySequence sussyBaka = bot.trajectorySequenceBuilder(backToPosition.end())
                .strafeRight(32) // direction of strafe depends what is considered the front/side of the bot
                .build();

            bot.followTrajectorySequence(sussyBaka);
        }
        else if (currentTag == 3) {
            TrajectorySequence jacobIsCute = bot.trajectorySequenceBuilder(backToPosition.end())
                .strafeRight(32)
                .build();

            bot.followTrajectorySequence(jacobIsCute);
        }

        telemetry.addData("currently in zone: ", tagUse);
        telemetry.update();

        while (opModeIsActive()){
            bot.update();
            armControl.update(telemetry);
            clawControl.update(); // we actually do need to update this. See the "update" method in the claw class for context
            slideControl.update(telemetry);
        }






























    }
}