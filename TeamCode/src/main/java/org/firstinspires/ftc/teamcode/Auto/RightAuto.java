package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;

@Autonomous(name = "Right Auto", group = "Auto")
public class RightAuto extends LinearOpMode {
    DcMotor Slide;
    ColorSensor color;
    TouchSensor limitSwitch;

    //Claw Mechanism
    Servo ClawX;
    Servo ClawY;
    private Encoder leftEncoder, rightEncoder, frontEncoder;


    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Slide = hardwareMap.dcMotor.get("Slide");
        Slide.setDirection(DcMotor.Direction.REVERSE);
        ClawX = hardwareMap.servo.get("ClawX");
        ClawY = hardwareMap.servo.get("ClawY");
        color = hardwareMap.get(ColorSensor.class, "color");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "left_drive"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "back_left_drive"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "right_drive"));



        leftEncoder.setDirection(Encoder.Direction.REVERSE);
        rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);






        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ReadyClaw();

        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence FirstCone = drive.trajectorySequenceBuilder(startPose)

                .addTemporalMarker(() -> SetSlideConditions ())


                .lineToLinearHeading(new Pose2d(59,0, Math.toRadians(0)))

                .waitSeconds(0.2)

                .back(7)
                .addTemporalMarker(() -> SlideTopPole ())
                .waitSeconds(0.5)


                .lineToLinearHeading(new Pose2d(52,15.3, Math.toRadians(0)))

                .forward(2.5)

                .addTemporalMarker(() -> ClawHold())
                .waitSeconds(0.1)

                .addTemporalMarker(() -> ClawDrop())
                .waitSeconds(0.5)


                .back(3)
                .addTemporalMarker(() -> ResetClaw())



                .build();
        TrajectorySequence ConePickup = drive.trajectorySequenceBuilder(FirstCone.end())
                .addTemporalMarker(() -> FirstCycle())

                .lineToLinearHeading(new Pose2d(52,-6,Math.toRadians(-83)))

                .forward(20)

                .addTemporalMarker(() -> ReadyClaw())
                .back(1)
                .addTemporalMarker(() -> UpCone())


                .build();


        TrajectorySequence  Cycle = drive.trajectorySequenceBuilder(ConePickup.end())


                .lineToLinearHeading(new Pose2d(52,0,Math.toRadians(0)))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawUp())
                .lineToLinearHeading(new Pose2d(52,15,Math.toRadians(0)))
                .addTemporalMarker(() -> SlideTopPole())
                .waitSeconds(0.75)
                .forward(2.3)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawHold())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawDrop())



                .build();
        TrajectorySequence Cyclep2 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(4.75)
                .addTemporalMarker(() -> SecondCycle())

                .lineToLinearHeading(new Pose2d(58,-22 ,Math.toRadians(-86)))

                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.2)
                .back(2)
                .addTemporalMarker(() -> UpCone())
                .waitSeconds(0.5)
                .back(27)
                .addTemporalMarker(() -> ResetSlide())
                .build();





        waitForStart();

        drive.followTrajectorySequence(FirstCone);
        telemetry.addData("LeftEncoder:", leftEncoder.getCurrentPosition());
        telemetry.addData("RightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("BackEncoder", frontEncoder.getCurrentPosition());
        telemetry.update();
        drive.followTrajectorySequence(ConePickup);
        telemetry.addData("LeftEncoder:", leftEncoder.getCurrentPosition());
        telemetry.addData("RightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("BackEncoder", frontEncoder.getCurrentPosition());
        telemetry.update();
        drive.followTrajectorySequence(Cycle );
        telemetry.addData("LeftEncoder:", leftEncoder.getCurrentPosition());
        telemetry.addData("RightEncoder", rightEncoder.getCurrentPosition());
        telemetry.addData("BackEncoder", frontEncoder.getCurrentPosition());
        telemetry.update();




    }
    private void SlideTopPole(){

        Slide.setTargetPosition(-3565);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }

    private void ResetSlide(){

        Slide.setTargetPosition(-10);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }
    private void FirstCycle() {
        Slide.setTargetPosition(-520);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Slide.isBusy()) {
            telemetry.addData("Don't let me down", "Im talking to you Dhruv");

        }

    } private void SecondCycle(){
        Slide.setTargetPosition(-450);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }
    private void UpCone(){
        Slide.setTargetPosition(-1400);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy())
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

    }

    private void ResetClaw(){

        ClawY.setPosition(0.73);
    }


    private void ReadyClaw(){
        ClawX.setPosition(0.83);
        ClawY.setPosition(0.73);
    }
    private void ClawHold(){

        ClawY.setPosition(0.85);
    }

    private void ClawDrop(){

        ClawX.setPosition(0.62);
    }
    private void ClawUp (){
        ClawY.setPosition(0.65);
    }

    private void SetSlideConditions(){
        Slide.setTargetPosition(-1000);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(-0.75);
        telemetry.addData("Slide", Slide.getCurrentPosition());
        telemetry.update();



    }
}
