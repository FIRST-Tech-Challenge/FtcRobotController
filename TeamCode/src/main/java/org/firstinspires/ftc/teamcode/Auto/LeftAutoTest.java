package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.Encoder;
@Disabled
@Autonomous(name = "Left+Auto", group = "Auto")
public class LeftAutoTest extends LinearOpMode {
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

                .addTemporalMarker(() -> SetSlideConditions())


                .lineToLinearHeading(new Pose2d(55,0, Math.toRadians(0)))
                .back(8)
                .waitSeconds(0.2)

                .turn(Math.toRadians(120))
                .forward(4.5)

                .addTemporalMarker(() -> ClawHold())
                .waitSeconds(0.1)

                .addTemporalMarker(() -> ClawDrop())
                .waitSeconds(0.1)


                .back(7)
                .addTemporalMarker(() -> ResetClaw())



                .build();
        TrajectorySequence ConePickup = drive.trajectorySequenceBuilder(FirstCone.end())
                .addTemporalMarker(() -> FirstCycle())

                .turn(Math.toRadians(-30))

                .lineToLinearHeading(new Pose2d(56,25,Math.toRadians(90)))


                .addTemporalMarker(() -> ReadyClaw())
                .back(1)
                .addTemporalMarker(() -> UpCone())


                .build();


        TrajectorySequence  Cycle = drive.trajectorySequenceBuilder(ConePickup.end())


                .back(7.5)
                .addTemporalMarker(() -> UpCone())
                .turn(Math.toRadians(110))

                .forward(5)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawHold())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawDrop())




                .build();
        TrajectorySequence Cyclep2 = drive.trajectorySequenceBuilder(Cycle.end())
                .back(5)
                .addTemporalMarker(() -> SecondCycle())
                .addTemporalMarker(() -> ResetClaw())

                .turn(Math.toRadians(-115))
                .forward(6)//6)


                .addTemporalMarker(() -> ReadyClaw())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> UpCone())
                .waitSeconds(0.5)
                .back(7)
                .turn(Math.toRadians(110))
                .forward(4.5)
                .addTemporalMarker(() -> ClawUp())
                .waitSeconds(0.1)
                .addTemporalMarker(() -> ClawDrop())


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
        drive.followTrajectorySequence(Cyclep2 );
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
        Slide.setTargetPosition(-490);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (Slide.isBusy()) {
            telemetry.addData("Don't let me down", "Im talking to you Dhruv");

        }

    } private void SecondCycle(){
        Slide.setTargetPosition(-400);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while(Slide.isBusy()){
            telemetry.addData("Don't let me down","Im talking to you Dhruv" );

        }
    }
    private void UpCone(){
        Slide.setTargetPosition(-1480);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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
        Slide.setTargetPosition(-1480);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(-0.75);
        telemetry.addData("Slide", Slide.getCurrentPosition());
        telemetry.update();



    }
}