package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.JackBurr.Drive.RobotConstantsV1;
import org.firstinspires.ftc.teamcode.JackBurr.Motors.DeliverySlidesV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryAxonV1;
import org.firstinspires.ftc.teamcode.JackBurr.Servos.DeliveryGrippersV1;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//TODO: PUSH FIRST THING
@Autonomous
public class RightAutoV2 extends LinearOpMode {
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
    @Override
    public void runOpMode() throws InterruptedException {
        //Pick SampleMecanumDrive for dashboard and RRMecanumDrive for no dashboard
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        deliveryAxon.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        deliveryGrippers.init(hardwareMap, telemetry);
        leftSlideTarget = constants.LEFT_SLIDE_HIGH_BAR_AUTO;
        rightSlideTarget = constants.RIGHT_SLIDE_HIGH_BAR_AUTO;

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        Pose2d startPose = new Pose2d(60, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeTo(new Vector2d(30, -20))
                .addDisplacementMarker(()->{
                    deliveryAxon.setPosition(constants.DELIVERY_HIGH_BAR);
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_GRAB);
                })
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(10)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .strafeTo(new Vector2d(60, 64))
                .addDisplacementMarker(()->{
                    timerStep03.reset();
                })
                .build();


        waitForStart();
        while (opModeIsActive()) {
            if (isStopRequested()) return;
            if (step == 1) {
                if(!deliveryAxonSet01){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_GRAB);
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
                    drive.followTrajectory(traj1);
                    traj1Followed = true;
                }
                else if(traj1Followed && timerStep03.seconds() > 2){
                    deliveryGrippers.setPosition(constants.DELIVERY_GRIPPERS_OPEN);
                    deliveryGrippersOpen = true;
                }
                if(traj1Followed && deliveryGrippersOpen) {
                    step = 3;
                }
            }
            else if(step == 3) {
                drive.followTrajectory(traj2);
                step = 4;
            }
            else if(step == 4) {
                drive.followTrajectory(traj3);
                step = 5;
            }
        }

        //       x->
        //      y
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
}
