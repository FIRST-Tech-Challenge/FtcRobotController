package org.firstinspires.ftc.teamcode.autoshellclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.RedBasketPose;
import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.SpecimenPose;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.OpModeType;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PinpointDrive;

import java.util.Locale;


@Config
@Autonomous(name = "Specimen_Auto", group = "Autonomous")
public class Specimen_Auto extends LinearOpMode {
    @Override
    public void runOpMode() {
        //Initialization steps
        PoseStorage.previousOpMode = OpModeType.AUTONOMOUS;
        PoseStorage.currentPose = RedBasketPose.basket_init_old; //This is to reset the pose te a default value that is not affected by where the auto ends so that teleop opmodes can be debugged
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Initializes Pinpoint
        Pose2d initialPose = new Pose2d(14.5, -63, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());
        //raises wrist on init
        Actions.runBlocking(_WristClawActions.WristUp());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;
        // Declare OpMode member for the Odometry Computer
        GoBildaPinpointDriverRR odo;

        Action driveToClip1, clippingSpecimen1, driveForSamplePush, driveToClip2, clippingSpecimen2, grabSpecimen3, driveToClip3, clippingSpecimen3, driveToPark, clawCloseWait1, clawCloseWait2;

        driveToClip1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(SpecimenPose.clipCenter.position, SpecimenPose.clipCenter.heading)
                .build();
        driveToClip2 = drive.actionBuilder(new Pose2d(46,-60, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(2, -48), Math.toRadians(90))
                .build();
        driveToClip3 = drive.actionBuilder(new Pose2d(38,-58,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(4,-48),Math.toRadians(90))
                .build();

        clippingSpecimen1 = drive.actionBuilder(new Pose2d(0,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-35.5),Math.toRadians(90), new TranslationalVelConstraint(25))
                .build();
        clippingSpecimen2 = drive.actionBuilder(new Pose2d(2,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(2,-35.5),Math.toRadians(90), new TranslationalVelConstraint(25))
                .build();
        clippingSpecimen3 = drive.actionBuilder(new Pose2d(4,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(4,-35.5),Math.toRadians(90), new TranslationalVelConstraint(25))
                .build();

        driveForSamplePush = drive.actionBuilder(new Pose2d(0,-36,Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38,-26, Math.toRadians(90) ), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(46,-12,Math.toRadians(0)), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(46,-60),Math.toRadians(0))//inner sample to observation
//                .strafeToLinearHeading(new Vector2d(46,-24),Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(54,-12,Math.toRadians(90)),Math.toRadians(0))//apex of middle sample
//                .splineToLinearHeading(new Pose2d(58, -16,Math.toRadians(90)),Math.toRadians(-90))
//                .strafeToLinearHeading(new Vector2d(58,-55),Math.toRadians(90))//middle sample to observation
//                .strafeToLinearHeading(new Vector2d(48,-58),Math.toRadians(-148))
//                .turnTo(Math.toRadians(-135))
                .build();

        grabSpecimen3 = drive.actionBuilder(new Pose2d(2,-36,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .build();

        driveToPark = drive.actionBuilder(new Pose2d(4,-36,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60,-60),Math.toRadians(90))
                .build();

        clawCloseWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();
        clawCloseWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        driveToClip1,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen1,
                        _ViperArmActions.LowerFromClip(),
                        driveForSamplePush,
                        _WristClawActions.CloseClaw(),
                        clawCloseWait1,
                        _WristClawActions.WristUp(),
                        driveToClip2,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen2,
                        _ViperArmActions.LowerFromClip(),
                        grabSpecimen3,
                        _WristClawActions.CloseClaw(),
                        clawCloseWait2,
                        _WristClawActions.WristUp(),
                        driveToClip3,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen3,
                        _ViperArmActions.LowerFromClip(),
                        driveToPark
                )
        );
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        PoseStorage.currentPose = odo.getPositionRR(); //save the pose for teleop
        telemetry.addData("Stored Pose: ", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, Math.toDegrees(PoseStorage.currentPose.heading.toDouble())) );

        while(opModeIsActive()) {
            // _leftFront.setPower(0.3);
            telemetry.update();
        }
    }
}
