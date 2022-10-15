package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Hardware9010;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

//435 max ticks per second is 383.6
@Autonomous(name = "RedAutono")
public class RedAutono extends LinearOpMode {

    Hardware9010 hdw;
    MecanumWheels robot;
    int steps = 0;
    private ElapsedTime runtime = new ElapsedTime();
    OpenCvWebcam webcam;
    BarcodeDeterminationRed.BarcodeDeterminationPipeline pipeline;
    BarcodeDeterminationRed.BarcodeDeterminationPipeline.BarcodePosition snapshotAnalysis = BarcodeDeterminationRed.BarcodeDeterminationPipeline.BarcodePosition.LEFT; // default

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        hdw = new Hardware9010(hardwareMap); //init hardware
        hdw.createHardware();
        robot = new MecanumWheels();

        hdw.wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontRight.setDirection(DcMotor.Direction.FORWARD);
        hdw.wheelBackRight.setDirection(DcMotor.Direction.FORWARD);
        hdw.wheelBackLeft.setDirection(DcMotor.Direction.REVERSE);
        hdw.wheelFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new BarcodeDeterminationRed.BarcodeDeterminationPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320,240, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {}
        });
        
        boolean shooterEnable = false;
        boolean TurretLeft = false;
        boolean TurretRight = false;
        boolean TurretUp = false;
        boolean TurretDown = false;
        boolean SpinnerLeft = false;
        boolean SpinnerRight = false;
        boolean EncoderUp = false;
        boolean EncoderDown = false;
        boolean fakeauto = true;
        boolean Turret = false;
        boolean Vertical = true;
        boolean autolevel = false;
        boolean autorotate = false;
        boolean intakeopen = false;
        boolean ManTurretRight = false;
        boolean ManTurretLeft = false;
        boolean ManTurretUp = false;
        boolean ManTurretDown = false;
        boolean TargetingOn = false;
        boolean TargetingOff = false;
        boolean TargetingRight = false;
        boolean TargetingLeft = false;
        boolean LevelUp = false;
        boolean LevelMid = false;
        boolean LevelDown = false;
        boolean LevelOff = false;
        boolean IntakeIn = false;
        boolean IntakeOut = false;
        boolean highmode = true;
        boolean sharemode = false;
        boolean slowDriveMode = false;
        boolean slowShootMode = false;
        boolean CarouselSpinner = false;
        boolean openclaw = false;
        boolean timer = true;
        boolean main = true;
        boolean first = true;
        boolean finalpass = true;
        int close = 0;
        int currentSlide = 0;
        double powerDrivePercentage = 1.0;
        double powerShootPercentage = 100;
        double offsetwheels = 0.0;

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        // Put initialization blocks here.
        hdw.grabberclaw.setPosition(1.0);
        hdw.Encoders.setPosition(0.35);

        while (!isStarted() && !isStopRequested())
        {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }
        
        waitForStart();
        
        snapshotAnalysis = pipeline.getAnalysis();
        telemetry.addData("Snapshot post-START analysis", snapshotAnalysis);
        telemetry.update();
        
        first = true;
        FirstCycle();
        MainCycle();
        main = true;
        MainCycle();
        //main = true;
        //MainCycle();
        finalpass = true;
        FinalCycle();
        
        /*
        AllTurret(500, 1.0, 2000, 1.0, 1100, 0.65);
        hdw.grabberclaw.setPosition(0.7);
        openclaw = true;
        sleep(150);
        SlideBack(0, 1.0, 0, 1.0, 1100, 0.65);
        SlideHome(0, 1.0, 0, 1.0, 0, 0.65);
        DriveForward(0.4, 1200);
        AutoCollection(0.2, 0);
        hdw.grabberclaw.setPosition(1.0);
        sleep(150);
        DriveReverse(-0.4, -500, 600, 0.4);
        ResetEncoder();
        AllTurret(500, 1.0, 2000, 1.0, 1100, 0.65);
        //AllTurret(500, 1.0, 2000, 1.0, 1100, 0.65);
        */
    }

    /**
     * Describe this function...
     */
    private void MainCycle()
    {
        steps = steps + 1;
        boolean main = true;
        hdw.grabberclaw.setPosition(0.85);
        boolean openclaw = true;
        sleep(150);
        SlideBack(0, 0.8, 0, 1.0, 1350, 1.0);
        SlideHome(0, 0.8, 0, 1.0, 60, 0.7);
        sleep(300);
        DriveForward(0.5, 7300, steps * -50, 1.0, 0, 1.0, 100, 0.4);
        AutoCollection(0.2, 0, steps * -50, 1.0, 0, 1.0, 100, 0.4);
        hdw.grabberclaw.setPosition(1.0);
        sleep(500);
        DriveReverseTurret(-0.34, -200, -525, 0.8, 1900, 1.0, 1250, 0.65);
        ResetEncoder();
        main = false;
        while (opModeIsActive() && (main == true))
        {
            idle();
        }
    }
    private void FirstCycle()
    {
        boolean first = true;
        switch (snapshotAnalysis)
        {
            case LEFT:
            {
                // place on low level of hub
                FirstMove(-525, 1.0, 2200, 1.0, -800, 1.0);
                break;
            }

            case RIGHT:
            {
                // place on high level of hub
                FirstMove(-525, 1.0, 2500, 1.0, 125, 1.0);
                break;
            }

            case CENTER:
            {
                // place on mid level of hub
                FirstMove(-525, 1.0, 2300, 1.0, -425, 1.0);
                break;
            }
        }
        hdw.grabberclaw.setPosition(0.85);
        boolean openclaw = true;
        sleep(150);
        FirstSlideBack(0, 1.0, 700, 1.0, 125, 0.4);
        FirstSlideHome(0, 1.0, 700, 0.8, 1030, 0.4);
        sleep(150);
        FirstDriveForward(0.5, 6400, 50, 1.0, 700, 0.8, 1030, 0.65);
        FirstAutoCollection(0.2, 0, 50, 1.0, 700, 1.0, 1000, 0.65);
        ResetTurretEncoder();
        hdw.grabberclaw.setPosition(1.0);
        openclaw = false;
        sleep(500);
        DriveReverseTurret(-0.34, -200, -525, 0.8, 1900, 1.0, 1250, 0.65);
        ResetEncoder();
        first = false;
        while (opModeIsActive() && (first == true))
        {
            idle();
        }
    }
    private void FinalCycle()
    {
        boolean finalpass = true;
        hdw.grabberclaw.setPosition(0.8);
        boolean openclaw = true;
        sleep(150);
        SlideBack(0, 0.8, 0, 1.0, 1250, 0.7);
        SlideHome(0, 0.8, 0, 1.0, 60, 0.7);
        sleep(300);
        DriveForward(0.8, 8900, 0, 1.0, 0, 1.0, 100, 0.65);
        ResetEncoder();
        finalpass = false;
        while (opModeIsActive() && (finalpass == true))
        {
            idle();
        }
    }
    private void FirstMove(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Encoders.setPosition(0.35);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        while (opModeIsActive() && (hdw.Turret.isBusy()))
        {
            idle();
        }
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        while (opModeIsActive() &&  (hdw.Slide.isBusy())) //|| hdw.Vertical.isBusy()))  (hdw.Turret.isBusy() ||
        {
            idle();
        }
        sleep(150);
    }
    private void ResetEncoder()
    {
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void ResetTurretEncoder()
    {
        hdw.Vertical.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hdw.Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    private void AllTurret(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Encoders.setPosition(0.35);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.Slide.isBusy() || hdw.Turret.isBusy())) //||  || hdw.Vertical.isBusy()))
        {
            idle();
        }
    }
    private void AutoCollection(double driveSpeed, int close, int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(0.8);
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(-VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.sensorDistance.getDistance(DistanceUnit.CM) > 2.8))
        {
            idle();
        }
    }
    private void FirstAutoCollection(double driveSpeed, int close, int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(0.8);
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(-VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.sensorDistance.getDistance(DistanceUnit.CM) > 2.8))
        {
            idle();
        }
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
    }
    private void DriveForward(double driveSpeed, double driveEncoders, int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(0.8);
        
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        hdw.wheelFrontRight.setPower(-0.5);
        hdw.wheelBackRight.setPower(0.5);
        hdw.wheelFrontLeft.setPower(0.5);
        hdw.wheelBackLeft.setPower(-0.5); 
        sleep(300);
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0); 
        sleep(100);

        hdw.Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(-VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed*1.2);
        hdw.wheelFrontLeft.setPower(driveSpeed*1.2);
        hdw.wheelBackLeft.setPower(driveSpeed); 
        
        while (opModeIsActive() && (hdw.wheelFrontLeft.getCurrentPosition() > -driveEncoders))
        {
            telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
            telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
    }
    private void FirstDriveForward(double driveSpeed, int driveEncoders, int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(0.8);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        
        hdw.Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(-VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed*1.2);
        hdw.wheelFrontLeft.setPower(driveSpeed*1.2);
        hdw.wheelBackLeft.setPower(driveSpeed);
    
        while (opModeIsActive() && (hdw.wheelFrontLeft.getCurrentPosition() > -driveEncoders))
        {
            telemetry.addData("lfWheel", hdw.wheelFrontLeft.getCurrentPosition());
            telemetry.addData("rfWheel", hdw.wheelFrontRight.getCurrentPosition());
            telemetry.update();
            idle();
        }
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
    }
    private void DriveReverse(double driveSpeed, int driveEncoders, int VerticalTarget, double VerticalSpeed)
    {
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(1.0);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        hdw.wheelFrontRight.setPower(driveSpeed);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed);
        while (opModeIsActive() && (hdw.wheelFrontLeft.getCurrentPosition() < driveEncoders))
        {
            idle();
        }
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
    }
    
    private void DriveReverseTurret(double driveSpeed, int driveEncoders, int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Encoders.setPosition(0.35);
        hdw.grabberclaw.setPosition(1.0);
        hdw.wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontRight.setPower(driveSpeed*1.3);
        hdw.wheelBackRight.setPower(driveSpeed);
        hdw.wheelFrontLeft.setPower(driveSpeed);
        hdw.wheelBackLeft.setPower(driveSpeed*1.3);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        sleep(250);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);

        while (opModeIsActive() && (hdw.wheelFrontLeft.getCurrentPosition() < driveEncoders))
        {
            idle();
            
        }
        hdw.wheelFrontRight.setPower(0.0);
        hdw.wheelBackRight.setPower(0.0);
        hdw.wheelBackLeft.setPower(0.0);
        hdw.wheelFrontLeft.setPower(0.0);
        sleep(300);
    }
    
    private void FirstSlideBack(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                       int VerticalTarget, double VerticalSpeed)
    {
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.Turret.getCurrentPosition() < -155))
        {
            idle();
        }
    }
    private void FirstSlideHome(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(-VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.Slide.getCurrentPosition() > 900))
        {
            idle();
        }
    }
    private void SlideBack(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                       int VerticalTarget, double VerticalSpeed)
    {
        hdw.Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Vertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.Turret.getCurrentPosition() < -155))
        {
            idle();
        }
        hdw.Turret.setPower(0.0);
        hdw.Slide.setPower(0.0);
        hdw.Vertical.setPower(0.0);
    }
    private void SlideHome(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                           int VerticalTarget, double VerticalSpeed)
    {
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive() && (hdw.Slide.getCurrentPosition() > 1800))
        {
            idle();
        }
    }
    private void AllTurretHold(int TurretTarget, double TurretSpeed, int SlideTarget, double SlideSpeed,
                               int VerticalTarget, double VerticalSpeed) {
        hdw.Turret.setTargetPosition(TurretTarget);
        hdw.Turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Turret.setPower(TurretSpeed);
        hdw.Slide.setTargetPosition(SlideTarget);
        hdw.Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Slide.setPower(SlideSpeed);
        hdw.Vertical.setTargetPosition(VerticalTarget);
        hdw.Vertical.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hdw.Vertical.setPower(VerticalSpeed);
        while (opModeIsActive()) {
            idle();
        }
    }
}
