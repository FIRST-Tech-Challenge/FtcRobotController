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
@Autonomous(name = "4-Specimen_Auto", group = "Autonomous")
public class Test4_Specimen_Auto extends LinearOpMode {
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
        Actions.runBlocking(_WristClawActions.WristDown());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;
        // Declare OpMode member for the Odometry Computer
        GoBildaPinpointDriverRR odo;

        Action driveToClip1, clippingSpecimen1, sampleGrab1, sampleDrop1, sampleGrab2, sampleDrop2, grabSpecimen2, driveToClip2, clippingSpecimen2, grabSpecimen3, driveToClip3, clippingSpecimen3, grabSpecimen4, driveToClip4, clippingSpecimen4, driveToPark, clawCloseSpecimenWait1, clawCloseSpecimenWait2, clawCloseSpecimenWait3, clawCloseSampleWait1, clawCloseSampleWait2, clawOpenSampleWait1, clawOpenSampleWait2;

        driveToClip1 = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(SpecimenPose.clipCenter.position, SpecimenPose.clipCenter.heading)
                .build();
        driveToClip2 = drive.actionBuilder(new Pose2d(46,-60, Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(2, -48), Math.toRadians(90))
                .build();
        driveToClip3 = drive.actionBuilder(new Pose2d(38,-58,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(4,-48),Math.toRadians(90))
                .build();
        driveToClip4 = drive.actionBuilder(new Pose2d(38,-58,Math.toRadians(0)))
                .strafeToLinearHeading(new Vector2d(6,-48),Math.toRadians(90))
                .build();

        clippingSpecimen1 = drive.actionBuilder(drive.pose)//(new Pose2d(0,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(0,-34),Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        clippingSpecimen2 = drive.actionBuilder(new Pose2d(2,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(2,-34),Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        clippingSpecimen3 = drive.actionBuilder(new Pose2d(4,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(4,-34),Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();
        clippingSpecimen4 = drive.actionBuilder(new Pose2d(6,-48,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(6,-34),Math.toRadians(90), new TranslationalVelConstraint(20))
                .build();

        sampleGrab1 = drive.actionBuilder(new Pose2d(0,-34,Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(36,-40, Math.toRadians(45) ), Math.toRadians(90))
                .build();
        sampleGrab2 = drive.actionBuilder(new Pose2d(0,-34,Math.toRadians(-45)))
                .strafeToLinearHeading(new Vector2d(48,-40), Math.toRadians(45))
                .build();

        sampleDrop1 = drive.actionBuilder(new Pose2d(36,-40,Math.toRadians(45)))
                .turnTo(-45)
                .build();
        sampleDrop2 = drive.actionBuilder(new Pose2d(48,-40,Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(48,-60), Math.toRadians(-45))
                .build();

        grabSpecimen2 = drive.actionBuilder(new Pose2d(0,-34,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .build();

        grabSpecimen3 = drive.actionBuilder(new Pose2d(2,-34,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .build();
        grabSpecimen4 = drive.actionBuilder(new Pose2d(4,-34,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(38,-58), Math.toRadians(0))
                .build();

        driveToPark = drive.actionBuilder(new Pose2d(6,-34,Math.toRadians(90)))
                .strafeToLinearHeading(new Vector2d(60,-60),Math.toRadians(90))
                .build();

        clawCloseSpecimenWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();
        clawCloseSpecimenWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();
        clawCloseSpecimenWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.3)
                .build();

        clawCloseSampleWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();
        clawCloseSampleWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();

        clawOpenSampleWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();
        clawOpenSampleWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.2)
                .build();

        Actions.runBlocking(
                new SequentialAction(
                        _WristClawActions.WristMid(),
                        _ViperArmActions.MoveArmToSpecimen(),
                        _ViperArmActions.MoveViperToSpecimen(),
                        clippingSpecimen1,
                        _ViperArmActions.LowerFromClip(),
                        sampleGrab1,
                        _ViperArmActions.MoveViperHalfExtend(),
                        _WristClawActions.CloseClaw(),
                        sampleDrop1,
                        _WristClawActions.OpenClaw(),
                        sampleGrab2,
                        _WristClawActions.CloseClaw(),
                        _ViperArmActions.MoveViperToHome(),
                        sampleDrop2,
                        _WristClawActions.OpenClaw(),
                        grabSpecimen2,
                        _WristClawActions.CloseClaw(),
                        clawCloseSpecimenWait1,
                        _WristClawActions.WristUp(),
                        driveToClip2,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen2,
                        _ViperArmActions.LowerFromClip(),
                        grabSpecimen3,
                        _WristClawActions.CloseClaw(),
                        clawCloseSpecimenWait2,
                        _WristClawActions.WristUp(),
                        driveToClip3,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen3,
                        _ViperArmActions.LowerFromClip(),
                        grabSpecimen4,
                        _WristClawActions.CloseClaw(),
                        driveToClip4,
                        _ViperArmActions.RaiseToClip(),
                        clippingSpecimen4,
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
