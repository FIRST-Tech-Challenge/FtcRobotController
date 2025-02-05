package org.firstinspires.ftc.teamcode.autoshellclasses;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders.BlueBasketPose;
import org.firstinspires.ftc.teamcode.BBcode.OpModeType;
import org.firstinspires.ftc.teamcode.BBcode.UtilClasses.UtilActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.ViperArmActions;
import org.firstinspires.ftc.teamcode.BBcode.MechanismActionBuilders.WristClawActions;
import org.firstinspires.ftc.teamcode.BBcode.PoseStorage;
import org.firstinspires.ftc.teamcode.PinpointDrive;
import java.util.Locale;


@Config
@Autonomous(name = "Blue_Basket_Auto", group = "Autonomous")
public class Blue_Basket_Auto extends LinearOpMode {
    //Pull in the values from the class that both TeamCode and MeepMeep can access and in a way that the values can be modified in FTCDesktop
    private static Pose2d pose_init = BlueBasketPose.init;
    public static double pose_init_x = BlueBasketPose.init.position.x;
    public static double pose_init_y = BlueBasketPose.init.position.y;
    public static double pose_init_h_deg = Math.toDegrees(pose_init.heading.toDouble());
    private static Pose2d pose_drop = BlueBasketPose.drop;
    public static double pose_drop_x = BlueBasketPose.drop.position.x;
    public static double pose_drop_y = BlueBasketPose.drop.position.y;
    public static double pose_drop_h_deg = Math.toDegrees(pose_drop.heading.toDouble());

    @Override
    public void runOpMode() {
    //Initialization steps
        PoseStorage.previousOpMode = OpModeType.AUTONOMOUS;
        //Creates instance of MechanismActionBuilders
        WristClawActions _WristClawActions = new WristClawActions(this);
        ViperArmActions _ViperArmActions = new ViperArmActions(this);

        //Write the public FTCDesktop static fields back into the private static poses so FTCDesktop actually affects the values on restart of op mode
        pose_init = new Pose2d(pose_init_x, pose_init_y, Math.toRadians(pose_init_h_deg));
        pose_drop = new Pose2d(pose_drop_x, pose_drop_y, Math.toRadians(pose_drop_h_deg));

        //Initializes Pinpoint
        PinpointDrive drive = new PinpointDrive(hardwareMap, pose_init);

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

        // Declare OpMode member for the Odometry Computer
        GoBildaPinpointDriverRR odo;

        //drive to drop
        Action driveToDropFromOgStart = drive.actionBuilder(BlueBasketPose.basket_init_old)
                .setTangent(-45)
                .splineToLinearHeading(pose_drop, 0)
                .build();
        Action driveToDropFromStart = drive.actionBuilder(pose_init)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        Action driveToDropFromInnerSample = drive.actionBuilder(BlueBasketPose.inner_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        Action driveToDropFromMiddleSample = drive.actionBuilder(BlueBasketPose.middle_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        Action driveToDropFromOuterSample = drive.actionBuilder(BlueBasketPose.outer_sample)
                .strafeToLinearHeading(pose_drop.position, pose_drop.heading)
                .build();

        //sample pickup
        Action samplePickupInner = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasketPose.inner_sample.position, BlueBasketPose.inner_sample.heading)
                .build();
        Action samplePickupMiddle = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasketPose.middle_sample.position, BlueBasketPose.middle_sample.heading)
                .build();
        Action samplePickupOuter = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasketPose.outer_sample.position, BlueBasketPose.outer_sample.heading)
                .build();

        //drive to park
        Action driveToPark = drive.actionBuilder(pose_drop)
                .strafeToLinearHeading(BlueBasketPose.park.position, BlueBasketPose.park.heading)
                .build();

    //----------------------------------------------------------------------------------------------
        Actions.runBlocking(
            new SequentialAction(
                _WristClawActions.CloseClaw(),
                _WristClawActions.WristUp(),
                UtilActions.Wait(0.25),
                driveToDropFromStart,
                _ViperArmActions.DumpInHighBasket(),
                samplePickupOuter,
                _ViperArmActions.MoveViperToSamplePickUp(),
                UtilActions.Wait(2)

//                driveToDropFromOuterSample,
//                samplePickupMiddle,
//                driveToDropFromMiddleSample,
//                samplePickupInner,
//                driveToDropFromInnerSample,
//                driveToPark

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
