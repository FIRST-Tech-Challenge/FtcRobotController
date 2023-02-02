package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.openCV.SignalDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(group = "autonomous", name = "AutonomousEsther")
public class AutonomousEsther extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private SignalDetection pipeline = null;
    private String webcamName = "Webcam";
    private boolean useCameraStream = true;
    private OpenCvWebcam camera = null;
    private Telemetry.Item camTelemetry = null;
    private Telemetry.Item colorTelemetry = null;
    private Telemetry.Item timeTelemetry = null;
    private SignalDetection.Color color = null;
    private long msUntilDetected = 0;

    @Override
    public void runOpMode() {
        DcMotor armMotor1 = hardwareMap.dcMotor.get("arm1");
        DcMotor armMotor2 = hardwareMap.dcMotor.get("arm2");

//        public void slideMovement(int Ticks) {
//            armMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotor1.setTargetPosition(Ticks);
//            armMotor1.setPower(0.5);
//            armMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            armMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotor2.setTargetPosition(Ticks);
//            armMotor2.setPower(0.5);
//            armMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        Servo intake = hardwareMap.servo.get("intake");
        Servo axon1 = hardwareMap.servo.get("axon1");
        Servo axon2 = hardwareMap.servo.get("axon2");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(-45, 0, Math.toRadians(-45)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(0, -15, Math.toRadians(0)))
                .build();


        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if(isStopRequested()) return;

//        slideMovement(1000);

//        drive.followTrajectory(traj1);
//        drive.turn(Math.toRadians(45));
//        // lineair slides uit
//        // intake arm ding uit
////        intake.setPosition(1.0);
//        //lineair slides in
//        drive.turn(Math.toRadians(45));
//        drive.followTrajectory(traj2);
        }
    }