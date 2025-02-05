package org.firstinspires.ftc.teamcode.autoshellclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.SpecimenPose;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.PinpointDrive;


@Config
@Autonomous(name = "Red_Specimen_Auto", group = "Autonomous")
public class Red_Specimen_Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Initialization steps
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Initializes Pinpoint
        Pose2d initialPose = new Pose2d(14.5, -63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;

        Action driveToClip1, clippingSpecimen1, driveForSamplePush, driveToClip2, clippingSpecimen2, grabSpecimen3, driveToClip3, clippingSpecimen3, driveToPark, clawCloseWait1, clawCloseWait2;

        driveToClip1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(0,-48), Math.toRadians(90))
                .build();
        driveToClip2 = drive.actionBuilder(new Pose2d(51,-57, Math.toRadians(-135)))
                .strafeToLinearHeading(new Vector2d(2, -48), Math.toRadians(90))
                .build();
        driveToClip3 = drive.actionBuilder(new Pose2d(38,-58,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(4,-48),Math.toRadians(90))
                .build();

        clippingSpecimen1 = drive.actionBuilder(new Pose2d(0,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .build();
        clippingSpecimen2 = drive.actionBuilder(new Pose2d(2,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(2,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .build();
        clippingSpecimen3 = drive.actionBuilder(new Pose2d(4,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(4,-36),Math.toRadians(90), new TranslationalVelConstraint(10))
                .build();

        driveForSamplePush = drive.actionBuilder(new Pose2d(0,-36,Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38,-26, Math.toRadians(90) ), Math.toRadians(90))
                .splineToConstantHeading(SpecimenPose.apex_inner.position, Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(46,-55),Math.toRadians(90))//inner sample to observation
                .strafeToLinearHeading(new Vector2d(46,-24),Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(54,-12,Math.toRadians(90)),Math.toRadians(0))//apex of middle sample
                .splineToLinearHeading(new Pose2d(58, -16,Math.toRadians(90)),Math.toRadians(-90))
                .strafeToLinearHeading(new Vector2d(58,-55),Math.toRadians(90))//middle sample to observation
                .strafeToLinearHeading(new Vector2d(48,-58),Math.toRadians(-140))
                .build();

        grabSpecimen3 = drive.actionBuilder(new Pose2d(2,-36,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .build();

        driveToPark = drive.actionBuilder(new Pose2d(4,-36,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60,-60),Math.toRadians(90))
                .build();

        clawCloseWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();
        clawCloseWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        driveToClip1,
//                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen1,
////                        _ViperArmActions.LowerFromClip(),
                        driveForSamplePush,
//                        _WristClawActions.CloseClaw(),
//                        clawCloseWait1,
                        driveToClip2,
////                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen2,
////                        _ViperArmActions.LowerFromClip(),
                        grabSpecimen3,
//                        _WristClawActions.CloseClaw(),
////                        clawCloseWait2,
                        driveToClip3,
////                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen3,
////                        _ViperArmActions.LowerFromClip(),
                        driveToPark
                )
        );
        while(opModeIsActive()) {
            // _leftFront.setPower(0.3);
            telemetry.update();
        }
    }
}
