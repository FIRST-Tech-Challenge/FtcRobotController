package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.PIDs.MotorPID;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FF_AsyncTrajectorySequence_Auto extends LinearOpMode {
    private DcMotorEx intakeMotor, liftMotorLeft, liftMotorRight;
    private Servo armJoint, clawJoint, wristJoint;

    double targetEncoderPosition = 30;

    MotorPID motorPID = new MotorPID(1,1,1); // This is where we tune PID values

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
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

            // Updating motorPID
            double leftError = targetEncoderPosition - liftMotorLeft.getCurrentPosition();
            double rightError = targetEncoderPosition - liftMotorRight.getCurrentPosition();

            // Run the left motor through the PID
            double leftTunedPosition = motorPID.getTunedPosition(timer);

            // Run the right motor through the PID
            double rightTunedPosition = motorPID.getTunedPosition(timer);

            liftMotorRight.setPower(leftTunedPosition); // I'm like 99% sure this does not work ¯\_(ツ)_/¯
            liftMotorLeft.setPower(rightTunedPosition);

            liftMotorRight.setTargetPosition(30); // We want the lift to be the same height on both sides, not uneven
            liftMotorRight.setTargetPosition(30);
        }
    }

} // end of class
