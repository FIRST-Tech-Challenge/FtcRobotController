package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.PIDs.PIDController;
import org.firstinspires.ftc.teamcode.TeleOps.AprilTags.PowerPlay_AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class PP_TrajectorySequence_AprilTags_PID_Auto extends LinearOpMode
{
    private DcMotorEx intakeMotor, liftMotorLeft, liftMotorRight;
    private Servo armJoint, clawJoint, wristJoint;
    ElapsedTime timer = new ElapsedTime();

    //Declaring lift motor powers
    double power;

    // Declaring our motor PID for the lift; passing through our PID values
    PIDController motorPID = new PIDController(0,0,0, timer); // This is where we tune PID values

    @Override
    public void runOpMode()
    {
        waitForStart();

        // Motors \\
        liftMotorLeft = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorLeft");
        liftMotorRight = (DcMotorEx)hardwareMap.dcMotor.get("liftMotorRight");

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // RUN_USING_ENCODER limits motors to about 80% of their full potential
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servos \\
        armJoint = hardwareMap.get(Servo.class, "armJoint");
        clawJoint = hardwareMap.get(Servo.class, "clawJoint");
        wristJoint = hardwareMap.get(Servo.class, "wristJoint");

        // bot object created
        SampleMecanumDrive bot = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-12, -59, Math.toRadians(90)); // x, y, heading (angle in radians)
        bot.setPoseEstimate(startPose);

        // TRAJECTORY SEQUENCES \\
        TrajectorySequence firstMove = bot.trajectorySequenceBuilder(startPose)
                .forward(15)
                .build();

        //  FOLLOW TRAJECTORY METHOD CALLS  \\
        bot.followTrajectorySequenceAsync(firstMove);

        // LOOPS TO RUN ASYNC \\
        while (opModeIsActive()){
            bot.update();

            // Our motors will stop running exactly at position 100
            power = motorPID.update(100, intakeMotor.getCurrentPosition());

            liftMotorLeft.setPower(power);
            liftMotorRight.setPower(power);
        }
    }

}
