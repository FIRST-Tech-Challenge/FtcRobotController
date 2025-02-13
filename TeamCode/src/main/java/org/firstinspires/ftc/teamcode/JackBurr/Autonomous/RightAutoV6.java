package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.AxonCRBenchTestRegularServoClass;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.JackBurr.Odometry.PinpointDrive;

import java.util.Vector;

@Autonomous
public class RightAutoV6 extends LinearOpMode {
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
    public ElapsedTime deliveryTimer = new ElapsedTime();
    public int leftSlideTarget = 0;
    public int rightSlideTarget = 0;
    public PinpointDrive drive;
    public int slowSpeed = 30;
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        Pose2d startPose = new Pose2d(-10, 62, Math.toRadians(90));
        Vector2d position1 = new Vector2d(-5, 30);
        Vector2d position2 = new Vector2d(-5, 45);
        Vector2d position3 = new Vector2d(-38, 45);
        Vector2d position4 = new Vector2d(-38, 10);
        Vector2d position5 = new Vector2d(-44, 10);
        Vector2d position6 = new Vector2d(-44, 57);
        Vector2d position7 = new Vector2d(-44, 10);
        Vector2d position8 = new Vector2d(-54, 10);
        Vector2d position9 = new Vector2d(-54, 57);
        Vector2d position10 = new Vector2d(-54, 10);
        Vector2d position11 = new Vector2d(-62, 10);
        Vector2d position12 = new Vector2d(-62, 57);
        Vector2d position13 = new Vector2d(-44, 25);



        drive = new PinpointDrive(hardwareMap, startPose);
        deliveryAxon.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        leftSlideTarget = constants.LEFT_SLIDE_HIGH_BAR_AUTO;
        rightSlideTarget = constants.RIGHT_SLIDE_HIGH_BAR_AUTO;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        TranslationalVelConstraint constraint = new TranslationalVelConstraint(30);
        TrajectoryActionBuilder traj1Builder = drive.actionBuilder(startPose)
                .splineToConstantHeading(position1, startPose.heading);
        TrajectoryActionBuilder traj2Builder = traj1Builder.fresh()
                .splineToConstantHeading(position2, startPose.heading, constraint)
                .splineToConstantHeading(position3, startPose.heading)
                .splineToConstantHeading(position4, startPose.heading)
                .turn(Math.toRadians(182))
                .splineToConstantHeading(position5, Math.toRadians(272))
                .splineToConstantHeading(position6, Math.toRadians(272))
                .splineToConstantHeading(position7, Math.toRadians(272))
                .splineToConstantHeading(position8, Math.toRadians(272))
                .splineToConstantHeading(position9, Math.toRadians(272))
                .splineToConstantHeading(position10, Math.toRadians(272))
                .splineToConstantHeading(position11, Math.toRadians(272))
                .splineToConstantHeading(position12, Math.toRadians(272))
                .splineToConstantHeading(position13, Math.toRadians(272))
                .turn(Math.toRadians(-180));

        Action traj1 = traj1Builder.build();
        Action traj2 = traj2Builder.build();
        //Action traj3 = traj3Builder.build();
        //Action traj4 = traj4Builder.build();
        //Action traj5 = traj5Builder.build();



        waitForStart();
        deliveryTimer.reset();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (step == 1) {
                if(!deliveryAxonSet01){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                    deliveryAxonSet01 = true;
                }
                while (deliveryTimer.seconds() < 2){
                    deliverySlides.runLeftSlideToPositionPID(constants.LEFT_SLIDE_HIGH_BAR_AUTO);
                    deliverySlides.runRightSlideToPositionPID(constants.RIGHT_SLIDE_HIGH_BAR_AUTO);
                }
                step = 2;
            }
            else if(step == 2){
                if(!traj1Followed && !deliveryGrippersOpen){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_CLOSE);
                    Actions.runBlocking(traj1);
                    traj1Followed = true;
                }
                else if(traj1Followed && !deliveryGrippersOpen){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    if(timerStep03.seconds() > 2) {
                        deliveryGrippersOpen = true;
                        timerStep03.reset();
                    }
                }
                if(deliveryGrippersOpen) {
                    while (timerStep03.seconds() < 2) {
                        deliverySlides.runLeftSlideToPositionPID(0);
                        deliverySlides.runRightSlideToPositionPID(0);
                        deliveryAxon.setPosition(constants.DELIVERY_GRAB);
                    }
                    if(traj1Followed && deliveryGrippersOpen) {
                        step = 3;
                    }
                }
            }
            else if(step == 3) {
                Actions.runBlocking(traj2);
                step = 4;
            }
            else if(step == 4) {
                // Actions.runBlocking(traj3);
                step = 5;
            }
            else if(step == 5){
                //Actions.runBlocking(traj4);
                step = 6;
            }
            else if(step == 6){
                //Actions.runBlocking(traj5);
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
