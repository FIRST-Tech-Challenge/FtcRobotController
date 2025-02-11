package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.IntakeSlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PinpointDrive;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DifferentialV2;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.GrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.WristAxonV1;

@Config
@Autonomous
public class LeftAutoV6 extends LinearOpMode {
    public DeliverySlidesV1 slides = new DeliverySlidesV1();
    public DeliveryAxonV1 deliveryAxonV1 = new DeliveryAxonV1();
    public RobotConstantsV1 robotConstantsV1 = new RobotConstantsV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public DifferentialV2 diffV2 = new DifferentialV2();
    public IntakeSlidesV1 intakeSlides = new IntakeSlidesV1();
    public WristAxonV1 wrist = new WristAxonV1();
    public GrippersV1 grippers = new GrippersV1();

    public ElapsedTime deliveryTimer = new ElapsedTime();
    public ElapsedTime intakeTimer = new ElapsedTime();
    public int step = 0;
    public static int intakeTarget01 = 401;
    public boolean servoSet = false;
    public boolean servoSet2 = false;
    public boolean wristSet = false;
    public boolean traj4Followed = false;
    public boolean traj5Followed = false;
    //----------------------------------------------------------------------------------------
    public static Pose2d startPose = new Pose2d(36, 62, Math.toRadians(-90)); // (60,0), 180

    public static Vector2d position1 = new Vector2d(51, 47);
    public static double position1HeadingDegrees = -135;

    public static Vector2d position2 = new Vector2d(47, 43);
    public static double position2HeadingDegrees = -85;

    public static Vector2d position4 = new Vector2d(61, 47);


    //+X is left, +Y is backwards


    //----------------------------------------------------------------------------------------

    public PinpointDrive drive;
    public TrajectoryActionBuilder traj1Builder;
    public TrajectoryActionBuilder traj2Builder;
    public TrajectoryActionBuilder traj3Builder;
    public TrajectoryActionBuilder traj4Builder;
    public TrajectoryActionBuilder traj5Builder;
    public TrajectoryActionBuilder traj6Builder;
    public TrajectoryActionBuilder traj7Builder;
    public Action traj1;
    public Action traj2;
    public Action traj3;
    public Action traj4;
    public Action traj5;
    public Action traj6;
    public Action traj7;

    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        drive = new PinpointDrive(hardwareMap, startPose);
        deliveryAxonV1.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        slides.init(hardwareMap);
        grippers.init(hardwareMap, telemetry);
        diffV2.init(hardwareMap, telemetry);
        intakeSlides.init(hardwareMap);
        wrist.init(hardwareMap);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        traj1Builder = drive.actionBuilder(startPose)
                .strafeTo(position1)
                .turnTo(Math.toRadians(position1HeadingDegrees));
        traj2Builder = traj1Builder.fresh()
                .strafeTo(position2)
                .turnTo(Math.toRadians(position2HeadingDegrees))
                .turnTo(Math.toRadians(position2HeadingDegrees))
                .strafeTo(position2);
        traj3Builder = traj2Builder.fresh()
                .strafeTo(position1)
                .turnTo(Math.toRadians(position1HeadingDegrees));
        traj4Builder = traj3Builder.fresh()
                .strafeTo(position4)
                .turnTo(Math.toRadians(position2HeadingDegrees))
                .turnTo(Math.toRadians(position2HeadingDegrees))
                .strafeTo(position2);

        //traj3Builder = traj2Builder.fresh()
        //.turnTo(Math.toRadians(position3Degrees));
        //traj4Builder = traj3Builder.fresh()
        //.strafeTo(position5);
        //traj5Builder = traj4Builder.fresh()
        //.turnTo(Math.toRadians(position4Degrees));
        //traj6Builder = traj4Builder.fresh()
        //.strafeTo(position1)
        //.turnTo(Math.toRadians(position6HeadingDegrees));
        //traj7Builder = traj6Builder.fresh()
        //.strafeTo(position6)
        //.turnTo(Math.toRadians(position6HeadingDegrees));





        traj1 = traj1Builder.build();
        traj2 = traj2Builder.build();
        traj3 = traj3Builder.build();
        traj4 = traj4Builder.build();
        //traj5 = traj5Builder.build();
        //traj6 = traj6Builder.build();
        //traj7 = traj7Builder.build();


        // Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
        //.back(15)
        //.build();

        //Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
        //.forward(15)
        //.build();

        //Trajectory traj4 = drive.trajectoryBuilder(traj3.path.)
        //.splineTo(new Vector2d(0, -15), Math.toRadians(-20))
        //.build();

        //Trajectory traj5 = drive.trajectoryBuilder(traj4.end())
        //.back(80)
        //.build();





        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
            if (step == 0) {
                deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                Actions.runBlocking(traj1);
                deliveryTimer.reset();
                step =  1;
            }
            if (step == 1){
                while (deliveryTimer.seconds() < 2){
                    deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                    slides.runLeftSlideToPosition(robotConstantsV1.LEFT_SLIDE_HIGH_BASKET, 0.9);
                    slides.runRightSlideToPosition(robotConstantsV1.RIGHT_SLIDE_HIGH_BASKET, 0.9);
                    if(!servoSet) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                        servoSet = true;
                    }
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                }
                while (deliveryTimer.seconds() < 3){
                    servoSet = false;
                    if(!servoSet2) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                        servoSet2 = true;
                    }
                }
                while (deliveryTimer.seconds() < 3.5){
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_OPEN);
                }
                while (deliveryTimer.seconds() < 4.5) {
                    intakeSlides.runToPosition(intakeTarget01, 1);
                    diffV2.setTopRightServoPosition(robotConstantsV1.FRONT_RIGHT_HOVER);
                    diffV2.setTopLeftServoPosition(robotConstantsV1.FRONT_LEFT_HOVER);
                    if (!servoSet) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                        grippers.setPosition(robotConstantsV1.GRIPPERS_OPEN);
                        servoSet = true;
                    }
                    if (slides.getLeftSlidePosition() != 0 || slides.getRightSlidePosition() != 0) {
                        slides.runLeftSlideToPosition(0, 0.9);
                        slides.runRightSlideToPosition(0, 0.9);
                    } else if (slides.getRightSlidePosition() != 0 && slides.getLeftSlidePosition() != 0) {
                        slides.runLeftSlideToPosition(0, 0.9);
                        slides.runRightSlideToPosition(0, 0.9);
                    }
                    if(!traj5Followed) {
                        Actions.runBlocking(traj2);
                        //Actions.runBlocking(traj4);
                        traj5Followed = true;
                    }
                }
                while (deliveryTimer.seconds() > 6 && servoSet) {
                    telemetry.addLine("HELLO");
                    servoSet = false;
                    intakeTimer.reset();
                    step = 2;
                }
            }
            if(step == 2) {
                while (intakeTimer.seconds() < 0.8){
                    diffV2.setTopRightServoPosition(robotConstantsV1.FRONT_RIGHT_PICKUP);
                    diffV2.setTopLeftServoPosition(robotConstantsV1.FRONT_LEFT_PICKUP);
                    if(!wristSet){
                        wrist.setPosition(robotConstantsV1.WRIST_CENTER);
                        wristSet = true;
                    }
                }
                while (intakeTimer.seconds() < 1.3){
                    grippers.setPosition(robotConstantsV1.GRIPPERS_GRAB);
                }
                step = 3;
            }
            if(step == 3) {
                //Actions.runBlocking(traj5);
                intakeTimer.reset();
                servoSet2 = false;
                step = 4;
            }
            if(step == 4) {
                while (intakeTimer.seconds() < 2){
                    if(!servoSet2){
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                    }
                    diffV2.setTopLeftServoPosition(robotConstantsV1.FRONT_LEFT_TRANSFER);
                    diffV2.setTopRightServoPosition(robotConstantsV1.FRONT_RIGHT_TRANSFER);
                }
                while (intakeTimer.seconds() < 3.5){
                    intakeSlides.intakeAllTheWayIn();
                }
                while (intakeTimer.seconds() < 4.2){
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                }
                while (intakeTimer.seconds() < 5){
                    grippers.setPosition(robotConstantsV1.GRIPPERS_OPEN);
                }
                Actions.runBlocking(traj3);
                //Actions.runBlocking(traj7);
                deliveryTimer.reset();
                servoSet = false;
                servoSet2 = false;
                //deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_LEVEL_ONE_ASCENT);
                step = 5;
            }
            if(step == 5){
                while (deliveryTimer.seconds() < 2){
                    deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                    slides.runLeftSlideToPosition(robotConstantsV1.LEFT_SLIDE_HIGH_BASKET, 0.9);
                    slides.runRightSlideToPosition(robotConstantsV1.RIGHT_SLIDE_HIGH_BASKET, 0.9);
                    if(!servoSet) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                        servoSet = true;
                    }
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_CLOSE);
                }
                while (deliveryTimer.seconds() < 2.75) {
                    servoSet = false;
                    if (!servoSet2) {
                        deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_UP);
                        servoSet2 = true;
                    }
                }
                deliveryTimer.reset();
                step = 6;
            }
            if(step == 6){
                while (deliveryTimer.seconds() < 1) {
                    deliveryGrippers.setPosition(robotConstantsV1.DELIVERY_GRIPPERS_OPEN);
                }
                servoSet2 = false;
                step = 7;
            }
            if(step == 7){
                if(!servoSet2) {
                    deliveryAxonV1.setPosition(robotConstantsV1.DELIVERY_GRAB);
                }
                if (slides.getLeftSlidePosition() != 0 || slides.getRightSlidePosition() != 0) {
                    slides.runLeftSlideToPosition(0, 0.9);
                    slides.runRightSlideToPosition(0, 0.9);
                } else if (slides.getRightSlidePosition() != 0 && slides.getLeftSlidePosition() != 0) {
                    slides.runLeftSlideToPosition(0, 0.9);
                    slides.runRightSlideToPosition(0, 0.9);
                }
                intakeSlides.runToPosition(intakeTarget01, 1);
                diffV2.setTopRightServoPosition(robotConstantsV1.FRONT_RIGHT_HOVER);
                diffV2.setTopLeftServoPosition(robotConstantsV1.FRONT_LEFT_HOVER);
                step = 7;
            }
            if(step == 8){
                Actions.runBlocking(traj4);
                wristSet = false;
                intakeTimer.reset();
                step = 8;
            }
            if(step == 9){
                while (intakeTimer.seconds() < 0.8){
                    diffV2.setTopRightServoPosition(robotConstantsV1.FRONT_RIGHT_PICKUP);
                    diffV2.setTopLeftServoPosition(robotConstantsV1.FRONT_LEFT_PICKUP);
                    if(!wristSet){
                        wrist.setPosition(robotConstantsV1.WRIST_CENTER);
                        wristSet = true;
                    }
                }
                while (intakeTimer.seconds() < 1.3){
                    grippers.setPosition(robotConstantsV1.GRIPPERS_GRAB);
                }
                step = 10;
            }
        }
    }
}
