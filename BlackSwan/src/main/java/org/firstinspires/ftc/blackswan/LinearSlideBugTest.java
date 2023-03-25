package org.firstinspires.ftc.blackswan;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.blackswan.drive.SampleMecanumDrive;
import org.firstinspires.ftc.blackswan.trajectorySequence.TrajectorySequence;

@Autonomous(name = "LSBUGTEST")
public class LinearSlideBugTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap, this);

        drive.closeClaw();
        drive.setArm();

        waitForStart();

        Pose2d startPose = new Pose2d(0 , 0,0);

        drive.setPoseEstimate(startPose);

        if (!isStopRequested()) {
            drive.liftDaBoi();
            while(opModeIsActive() && drive.linearslide.isBusy()){

            }
            drive.numberTheArm(1000);
            while(opModeIsActive() && drive.linearslide.isBusy()){

            }
            drive.pause(10000);
        }
    }
}
