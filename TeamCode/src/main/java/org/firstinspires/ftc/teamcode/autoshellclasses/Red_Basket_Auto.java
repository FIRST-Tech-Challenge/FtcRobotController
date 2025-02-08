package org.firstinspires.ftc.teamcode.autoshellclasses;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.BBcode.OpModeType;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.UtilActions;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import java.util.Locale;


@Config
@Autonomous(name = "Red_Basket_Auto", group = "Autonomous")
public class Red_Basket_Auto extends LinearOpMode {

    @Override
    public void runOpMode() {
        //Initialization steps
        PoseStorage.previousOpMode = OpModeType.AUTONOMOUS;
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Write the public FTCDesktop static fields back into the private static poses so FTCDesktop actually affects the values on restart of op mode
        Pose2d initialPose = new Pose2d(-47, -59.5, Math.toRadians(45));
//        pose_drop = new Pose2d(pose_drop_x, pose_drop_y, Math.toRadians(pose_drop_h_deg));

        //Initializes Pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());
        Actions.runBlocking(_WristClawActions.WristUp());

        telemetry.update();
        waitForStart();
        //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;


        // Declare OpMode member for the Odometry Computer
        GoBildaPinpointDriverRR odo;

        //drive to drop
        Action driveToDropFromStart = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(-55.9, -55.275), Math.toRadians(45))
                .build();

        Action driveToDropFromInnerSample = drive.actionBuilder(new Pose2d(-44.60, -43.59, Math.toRadians(102.8)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-55.9, -55.275), Math.toRadians(45))
                .build();

        Action driveToDropFromMiddleSample = drive.actionBuilder(new Pose2d(-51.70, -43.05, Math.toRadians(113.59)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-55.9, -55.275), Math.toRadians(45))
                .build();

        Action driveToDropFromOuterSample = drive.actionBuilder(new Pose2d(-56.65, -39.40, Math.toRadians(127.75)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-55.9, -55.275), Math.toRadians(45))
                .build();

        //sample pickup
        Action samplePickupInner = drive.actionBuilder(new Pose2d(-55.9, -55.275, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-44.60, -43.59), Math.toRadians(102.8))
                .build();
        Action samplePickupMiddle = drive.actionBuilder(new Pose2d(-55.9, -55.275, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-51.70, -43.05), Math.toRadians(113.59))
                .build();
        Action samplePickupOuter = drive.actionBuilder(new Pose2d(-55.9, -55.275, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-56.65, -40.50), Math.toRadians(127.75))
                .build();

        //drive to park
        Action driveToPark = drive.actionBuilder(new Pose2d(-55.9, -55.275, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(38, -60), Math.toRadians(90))
                .build();

        //----------------------------------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        driveToDropFromStart,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupInner,
                        _WristClawActions.PickUpSample(),
                        driveToDropFromOuterSample,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupMiddle,
                        _WristClawActions.CloseClaw(),
                        _WristClawActions.PickUpSample(),
                        _WristClawActions.WristUp(),
                        driveToDropFromMiddleSample,
                        _ViperArmActions.DumpInHighBasketHalfExtend(),
                        samplePickupOuter,
                        _WristClawActions.CloseClaw(),
                        _WristClawActions.PickUpSample(),
                        _WristClawActions.WristUp(),
                        driveToDropFromInnerSample,
                        _ViperArmActions.DumpInHighBasket(),
                        _WristClawActions.WristUp(),
                        driveToPark
                )
        );
        odo = hardwareMap.get(GoBildaPinpointDriverRR.class,"pinpoint");
        PoseStorage.currentPose = odo.getPositionRR();
        telemetry.addData("Stored Pose: ", String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", PoseStorage.currentPose.position.x, PoseStorage.currentPose.position.y, Math.toDegrees(PoseStorage.currentPose.heading.toDouble())) );
        //odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        //odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
//        while(opModeIsActive()) {
//            // _leftFront.setPower(0.3);
//            odo.update();
//            Pose2d pos = odo.getPositionRR();
//            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.position.x, pos.position.y, Math.toDegrees(pos.heading.toDouble()));
//
//            telemetry.update();
//        }
    }
}
