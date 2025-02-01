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
import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.BlueBasket;
import org.firstinspires.ftc.teamcode.BBcode.AutoUtils;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import java.util.Locale;


@Config
@Autonomous(name = "Blue_Basket_Auto", group = "Autonomous")
public class Blue_Basket_Auto extends LinearOpMode {
    //Pull in the values from the class that both TeamCode and MeepMeep can access and in a way that the values can be modified in FTCDesktop
    private static Pose2d pose_init = BlueBasket.pose_init;
    public static double pose_init_x = BlueBasket.pose_init.position.x;
    public static double pose_init_y = BlueBasket.pose_init.position.y;
    public static double pose_init_h_deg = Math.toDegrees(pose_init.heading.toDouble());
    private static Pose2d pose_drop = BlueBasket.pose_drop;
    public static double pose_drop_x = BlueBasket.pose_drop.position.x;
    public static double pose_drop_y = BlueBasket.pose_drop.position.y;
    public static double pose_drop_h_deg = Math.toDegrees(pose_drop.heading.toDouble());

    @Override
    public void runOpMode() {
    //Initialization steps
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Write the public FTCDesktop static fields back into the private static poses so FTCDesktop actually affects the values on restart of op mode
        pose_init = new Pose2d(pose_init_x, pose_init_y, Math.toRadians(pose_init_h_deg));
        pose_drop = new Pose2d(pose_drop_x, pose_drop_y, Math.toRadians(pose_drop_h_deg));

        //Initializes Pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose_init);

        //Imports AutoUtils
        AutoUtils _AutoUtils = new AutoUtils(drive);

        //closes claw on init
        Actions.runBlocking(_WristClawActions.CloseClaw());

        telemetry.update();
        waitForStart();
    //----------------------------------------------------------------------------------------------

        if (isStopRequested()) return;

        Vector2d outer_sample_pickup_position = new Vector2d(46, 43.25);
        Vector2d middle_sample_pickup_position = new Vector2d(56, 43);
        double sample_pickup_heading = Math.toRadians(-90);
        Vector2d inner_sample_pickup_position = new Vector2d(57, 37.5);//57, 38.5
        double inner_sample_pickup_heading = Math.toRadians(-50);
        double rotation_speed = Math.toRadians(0.5);

        GoBildaPinpointDriverRR odo; // Declare OpMode member for the Odometry Computer
        Action driveToDropFromOgStart, driveToDropFromStart, driveToDropFromInnerSample, driveToDropFromMiddleSample, driveToDropFromOuterSample, samplePickupInner, samplePickupMiddle, samplePickupOuter, driveToPark, armUpWait2, armUpWait3, armUpWait4, armDownWait1, armDownWait2, armDownWait3, armDownWait4, viperUpWait1, viperUpWait2, viperUpWait3, viperUpWait4, viperDownWait1, viperDownWait2, viperDownWait3, viperDownWait4, wristUpWait1, wristUpWait2, wristUpWait3, wristDownWait1, wristDownWait2, wristDownWait3, wristDownWait4, clawOpenWait1, clawOpenWait2, clawOpenWait3, clawOpenWait4, clawCloseWait1, clawCloseWait2, clawCloseWait3, clawCloseWait4;
        //drive to drop
        driveToDropFromOgStart = drive.actionBuilder(BlueBasket.pose_basket_init_old)
                .setTangent(-45)
                .splineToLinearHeading(pose_drop, 0)
                .build();
        driveToDropFromStart = drive.actionBuilder(pose_init)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        driveToDropFromInnerSample = drive.actionBuilder(BlueBasket.pose_inner_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        driveToDropFromMiddleSample = drive.actionBuilder(BlueBasket.pose_middle_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        driveToDropFromOuterSample = drive.actionBuilder(BlueBasket.pose_outer_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        //sample pickup
        samplePickupInner = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasket.pose_inner_sample.position, BlueBasket.pose_inner_sample.heading)
                .build();
        samplePickupMiddle = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasket.pose_middle_sample.position, BlueBasket.pose_middle_sample.heading)
                .build();
        samplePickupOuter = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasket.pose_outer_sample.position, BlueBasket.pose_outer_sample.heading)
                .build();

        //drive to park
        driveToPark = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasket.pose_park.position, BlueBasket.pose_park.heading)
                .build();

//        armUpWait1 = drive.actionBuilder(drive.pose)
//                .waitSeconds(0.75)
//                .build();
        armUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();
        armUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();
        armUpWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.75)
                .build();

        armDownWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        armDownWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        armDownWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        armDownWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        viperUpWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperUpWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        viperDownWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(5)
                .build();
        viperDownWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperDownWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        viperDownWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        wristUpWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristUpWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristUpWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        wristDownWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristDownWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristDownWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        wristDownWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        clawCloseWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawCloseWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawCloseWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawCloseWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();

        clawOpenWait1 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawOpenWait2 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawOpenWait3 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
        clawOpenWait4 = drive.actionBuilder(drive.pose)
                .waitSeconds(0.5)
                .build();
//        Action test = drive.actionBuilder(drive.pose)
//                        .turnTo(Math.toRadians(225))
//                        .build();

        Actions.runBlocking(
                new SequentialAction(
                        driveToDropFromStart,
                        _AutoUtils.Wait(2),
                        samplePickupOuter,
                        _AutoUtils.Wait(2),
                        driveToDropFromOuterSample,
                        _AutoUtils.Wait(2),
                        samplePickupMiddle,
                        _AutoUtils.Wait(2),
                        driveToDropFromMiddleSample,
                        _AutoUtils.Wait(2),
                        samplePickupInner,
                        _AutoUtils.Wait(2),
                        driveToDropFromInnerSample,
                        _AutoUtils.Wait(2),
                        driveToPark

//                        _ViperArmActions.MoveArmToHighBasket(),
//                        armUpWait1,
//                        _ViperArmActions.MoveViperToHighBasket(),
//                        viperUpWait1,
//                        _WristClawActions.OpenClaw(),
//                        clawOpenWait1,
//                        _WristClawActions.WristDown(),
//                        wristDownWait1,
//                        _ViperArmActions.MoveViperToHome(),
//                        viperDownWait1,
//                        _ViperArmActions.MoveArmToHome(),
//                        armDownWait1,
//                        samplePickup1,
//                        _WristClawActions.CloseClaw(),
//                        clawCloseWait1,
//                        _WristClawActions.WristUp(),
//                        wristUpWait1,
//                        driveToDrop2,
//                        _ViperArmActions.MoveArmToHighBasket(),
//                        armUpWait2,
//                        _ViperArmActions.MoveViperToHighBasket(),
//                        viperUpWait2,
//                        _WristClawActions.OpenClaw(),
//                        clawOpenWait2,
//                        _WristClawActions.WristDown(),
//                        wristDownWait2,
//                        _ViperArmActions.MoveViperToHome(),
//                        viperDownWait2,
//                        _ViperArmActions.MoveArmToHome(),
//                        armDownWait2,
//                        samplePickup2,
//                        _WristClawActions.CloseClaw(),
//                        clawCloseWait2,
//                        _WristClawActions.WristUp(),
//                        wristUpWait2,
//                        driveToDrop3,
//                        _ViperArmActions.MoveArmToHighBasket(),
//                        armUpWait3,
//                        _ViperArmActions.MoveViperToHighBasket(),
//                        viperUpWait3,
//                        _WristClawActions.OpenClaw(),
//                        clawOpenWait3,
//                        _WristClawActions.WristDown(),
//                        wristDownWait3,
//                        _ViperArmActions.MoveViperToHome(),
//                        viperDownWait3,
//                        _ViperArmActions.MoveArmToHome(),
//                        armDownWait3,
//                        samplePickup3,
//                        _WristClawActions.CloseClaw(),
//                        clawCloseWait4,
//                        _WristClawActions.WristUp(),
//                        wristUpWait3,
//                        driveToDrop4,
//                        _ViperArmActions.MoveArmToHighBasket(),
//                        armUpWait4,
//                        _ViperArmActions.MoveViperToHighBasket(),
//                        viperUpWait4,
//                        _WristClawActions.OpenClaw(),
//                        clawOpenWait4,
//                        _WristClawActions.WristDown(),
//                        wristDownWait4,
//                        _ViperArmActions.MoveViperToHome(),
//                        viperDownWait4,
//                        _ViperArmActions.MoveArmToHome(),
//                        armDownWait4,
//                        driveToPark
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
