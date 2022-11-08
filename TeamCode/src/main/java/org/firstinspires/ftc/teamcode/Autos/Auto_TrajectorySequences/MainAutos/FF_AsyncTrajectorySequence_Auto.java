package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences.MainAutos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FF_AsyncTrajectorySequence_Auto extends LinearOpMode {
    private DcMotorEx intakeMotor, liftMotorLeft, liftMotorRight;
    private Servo armJoint, clawJoint, wristJoint;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        double integralSum = 0;
        double error = 0;
        double lastError = 0;

        // This is where we tune our pid values
        double Kp = 0;
        double Ki = 0;
        double Kd = 0;

        int reference = 30;

        final int NUM_CYCLES = 3;
        final int stepIncrement = 0;

        // Motors
        intakeMotor = (DcMotorEx)hardwareMap.dcMotor.get("intakeMotor");
        liftMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorLeft");
        liftMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorRight");

        // Servos
        armJoint = hardwareMap.get(Servo.class, "armJoint");
        clawJoint = hardwareMap.get(Servo.class, "clawJoint");
        wristJoint = hardwareMap.get(Servo.class, "wristJoint");

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // bot object created
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        TrajectorySequence firstMove = bot.trajectorySequenceBuilder(startPose)
               .forward(15)
               .build();

        //  FOLLOW TRAJECTORY METHOD CALLS  \\
        bot.followTrajectorySequenceAsync(firstMove);

        waitForStart(); // Wait until player presses start button on DS

        while (opModeIsActive()){
            bot.update();
            // motorPID.update();
        }
    }

} // end of class
