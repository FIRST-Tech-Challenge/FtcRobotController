package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.BackupAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Arm;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Claw;
import org.firstinspires.ftc.teamcode.MechanismTemplates.Slide;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionDeposit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "BlueTerminal_Auto")
public class BlueTerminalAuto extends PowerPlay_AprilTagDetectionDeposit {
    private Arm armControl;
    private Slide slideControl;
    private Claw clawControl;

    public void initialize(){
        super.runOpMode();
    }

    @Override
    public void runOpMode() {
        int currentTag;
        armControl = new Arm(hardwareMap);
        slideControl = new Slide(hardwareMap);
        clawControl = new Claw(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        bot.setPoseEstimate(startPose);

        // TRAJECTORY SEQUENCES \\
        TrajectorySequence rotate = bot.trajectorySequenceBuilder(startPose)
            .forward(8) // in inches
            .turn(Math.toRadians(175))
            .UNSTABLE_addTemporalMarkerOffset(1, () -> {initialize();})
            .waitSeconds(2) // waits to scan AprilTag
            .build();

        TrajectorySequence forward_extake = bot.trajectorySequenceBuilder(rotate.end())
            .forward(30)
            .splineTo(new Vector2d(40, 5), Math.toRadians(140))
              /*  .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                slideControl.setHighJunction();
                armControl.setExtake();
                clawControl.toggleWristRotate();
                })*/
            .waitSeconds(1)
             /*   .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                clawControl.clawJoint.setPosition(clawControl.OPEN);
                armControl.setIntake();
                slideControl.setIntakeOrGround();
                clawControl.clawJoint.setPosition(clawControl.CLOSE);
                })*/
            .build();

        TrajectorySequence backToPosition = bot.trajectorySequenceBuilder(forward_extake.end())
                .lineToLinearHeading(new Pose2d(-35,35,Math.toRadians(175)))
                .build();

        //  FOLLOW TRAJECTORY METHOD CALLS  \\
        waitForStart();
        bot.followTrajectorySequence(rotate); // we want to follow this sync and not async because the bot needs to move immediately
        currentTag = tagUse;
        bot.followTrajectorySequenceAsync(forward_extake);
        //bot.followTrajectorySequenceAsync(backToPosition);

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

        while(opModeIsActive() && !isStopRequested()){
            bot.update();
            //armControl.update(telemetry);
            // clawControl.update(); we actually do need to update this. See the "update" method in the claw class for context
            //slideControl.update(telemetry);
        }
    }
}