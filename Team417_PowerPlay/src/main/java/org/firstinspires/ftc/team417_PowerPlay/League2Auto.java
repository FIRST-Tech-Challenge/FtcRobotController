package org.firstinspires.ftc.team417_PowerPlay;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.team417_PowerPlay.drive.SampleMecanumDrive;
@Disabled
@Autonomous (name = "league 2!!")
public class League2Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //initializeHardware();
        Servo grabberServo = hardwareMap.servo.get("grabberServo");
        grabberServo.setPosition(0.0);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        //DcMotor motorArm = hardwareMap.dcMotor.get("motorArm");
        //motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //motorArm.setPower(0);

        Trajectory traject2 = drive.trajectoryBuilder(startPose, false)
                .forward(48)
                .build();
        Trajectory traject3 = drive.trajectoryBuilder(traject2.end(), false)
                .back(24)
                .build();
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeRight(12)
                .build();
        /*Trajectory traject3 = drive.trajectoryBuilder(traject2.end().plus(new Pose2d(0,0, Math.toRadians(-90))), false)
                .strafeLeft(41)
                .build();
        // add arm to deliver cone
        Trajectory traject4 = drive.trajectoryBuilder(traject3.end(), false)
                .strafeLeft(12)
                .build();
        Trajectory traject5 = drive.trajectoryBuilder(traject4.end(), false)
                .forward(24)
                .build();*/
        // collect cone

        telemetry.addLine("Ready for start");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(traject2);
        drive.followTrajectory(traject3);
        /*motorArm.setTargetPosition(-1000);
        while (Math.abs(motorArm.getCurrentPosition() - -1000) > 20) {
            motorArm.setTargetPosition(-1000);

            motorArm.setPower((-1000 - motorArm.getCurrentPosition()) / 1000.0);
        }
        motorArm.setPower(0.05);*/
        drive.followTrajectory(traject4);

        telemetry.addData("pose", drive.getPoseEstimate());
        telemetry.update();
        sleep(20000);
    }
}