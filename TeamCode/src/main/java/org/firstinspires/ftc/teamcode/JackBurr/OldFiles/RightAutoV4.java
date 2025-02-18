package org.firstinspires.ftc.teamcode.JackBurr.OldFiles;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.Roadrunner.PinpointDrive;
@Disabled
@Autonomous
public class RightAutoV4 extends LinearOpMode {
    public int step = 1;
    public DeliveryAxonV1 deliveryAxon = new DeliveryAxonV1();
    public DeliverySlidesV1 deliverySlides = new DeliverySlidesV1();
    public RobotConstantsV1 constants = new RobotConstantsV1();
    public DeliveryGrippersV1 deliveryGrippers = new DeliveryGrippersV1();
    public boolean deliveryAxonSet01 = false;
    public boolean traj1Followed = false;
    public boolean deliveryGrippersOpen = false;
    public ElapsedTime timerStep01 = new ElapsedTime();
    public ElapsedTime timerStep03 = new ElapsedTime();
    public int leftSlideTarget = 0;
    public int rightSlideTarget = 0;
    public PinpointDrive drive;
    public int slowSpeed = 20;
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        Pose2d startPose = new Pose2d(60, 0, Math.toRadians(0));
        drive = new PinpointDrive(hardwareMap, startPose);
        deliveryAxon.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        leftSlideTarget = constants.LEFT_SLIDE_HIGH_BAR_AUTO;
        rightSlideTarget = constants.RIGHT_SLIDE_HIGH_BAR_AUTO;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        VelConstraint slow = new TranslationalVelConstraint(slowSpeed);
        TrajectoryActionBuilder traj1Builder = drive.actionBuilder(startPose)
            .splineToConstantHeading(new Vector2d(0, -50), startPose.heading, slow);
        TrajectoryActionBuilder traj2Builder = traj1Builder.fresh()
                 .strafeTo(new Vector2d(15, -50));
        TrajectoryActionBuilder traj3Builder = traj2Builder.fresh()
                .strafeTo(new Vector2d(40, 14));
        TrajectoryActionBuilder traj4Builder = traj3Builder.fresh()
                .splineToConstantHeading(new Vector2d(0, -20), startPose.heading);
        TrajectoryActionBuilder traj5Builder = traj4Builder.fresh()
                .turn(90);

        Action traj1 = traj1Builder.build();
        Action traj2 = traj2Builder.build();
        Action traj3 = traj3Builder.build();
        Action traj4 = traj4Builder.build();
        Action traj5 = traj5Builder.build();



        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (step == 1) {
                if(!deliveryAxonSet01){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                    deliveryAxonSet01 = true;
                }
                if(!isRightInRange(rightSlideTarget, 10)){
                    deliverySlides.runRightSlideToPosition(rightSlideTarget, 1);
                }
                if(!isLeftInRange(leftSlideTarget, 10)){
                    deliverySlides.runLeftSlideToPosition(leftSlideTarget, 1);
                }
                if(isLeftInRange(leftSlideTarget, 10) && isRightInRange(rightSlideTarget, 10) && timerStep01.seconds() > 5){
                    timerStep03.reset();
                    step = 2;
                }
            }
            else if(step == 2){
                if(!traj1Followed){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    Actions.runBlocking(traj1);
                    traj1Followed = true;
                }
                else if(traj1Followed){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    if(timerStep03.seconds() > 2) {
                        deliveryGrippersOpen = true;
                    }
                }
                if(traj1Followed && deliveryGrippersOpen) {
                    step = 3;
                }
            }
            else if(step == 3) {
                if(timerStep03.seconds() > 4) {
                    Actions.runBlocking(traj2);
                    step = 4;
                }
            }
            else if(step == 4) {
                Actions.runBlocking(traj3);
                step = 5;
            }
            else if(step == 5){
                Actions.runBlocking(traj4);
                step = 6;
            }
            else if(step == 6){
                Actions.runBlocking(traj5);
                step = 7;
            }
        }

        //       y->
        //      x
        //      |
        //

    }
    public boolean isLeftInRange(int target, int range){
        int leftRange = target - range;
        int rightRange = target + range;
        if(deliverySlides.getLeftSlidePosition() > leftRange && deliverySlides.getLeftSlidePosition() < rightRange){
            return true;
        }
        else if(deliverySlides.getLeftSlidePosition() == leftRange){
            return true;
        }
        else if(deliverySlides.getLeftSlidePosition() == rightRange){
            return true;
        }
        else {
            return false;
        }
    }

    public boolean isRightInRange(int target, int range){
        int leftRange = target - range;
        int rightRange = target + range;
        if(deliverySlides.getRightSlidePosition() > leftRange && deliverySlides.getRightSlidePosition() < rightRange){
            return true;
        }
        else if(deliverySlides.getRightSlidePosition() == leftRange){
            return true;
        }
        else if(deliverySlides.getRightSlidePosition() == rightRange){
            return true;
        }
        else {
            return false;
        }
    }

    public TrajectoryActionBuilder moveForward(Pose2d previousCoordinates, int distance, boolean slow){
        Vector2d newCoordinates = new Vector2d((previousCoordinates.position.x - distance), previousCoordinates.position.y);
        TrajectoryActionBuilder builder;
        if(slow) {
            builder = drive.actionBuilder(previousCoordinates)
                    .strafeTo(newCoordinates);
        }
        else {
            builder = drive.actionBuilder(previousCoordinates)
                    .strafeTo(newCoordinates, new TranslationalVelConstraint(slowSpeed));
        }
        return builder;

    }

}
