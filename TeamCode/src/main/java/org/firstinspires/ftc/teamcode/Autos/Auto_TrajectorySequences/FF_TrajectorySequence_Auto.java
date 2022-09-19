package org.firstinspires.ftc.teamcode.Autos.Auto_TrajectorySequences;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class FF_TrajectorySequence_Auto extends LinearOpMode {
    private DcMotorEx intakeMotor, liftMotorLeft, liftMotorRight;
    private Servo armJoint, clawJoint, wristJoint;

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

        TrajectorySequence openingMove = bot.trajectorySequenceBuilder(startPose)
                .forward(15)

                // the temporal marker below would run during the waitSeconds() following it
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    liftMotorLeft.setTargetPosition(30);
                    liftMotorRight.setTargetPosition(30);

                    clawJoint.setPosition(0);
                    // the curly braces {} allow you to make multiple statements in a marker
                    }
                )
                .waitSeconds(1.5)

            .build();

        TrajectorySequence cycles = bot.trajectorySequenceBuilder(openingMove.end())
            .lineToLinearHeading(new Pose2d(10,-63.5, Math.toRadians(180)))

                .addDisplacementMarker(() -> {
                            intakeMotor.setPower(1);
                        }
                )

            .back(25 + stepIncrement)

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    intakeMotor.setPower(0);
                   }
                )
            .waitSeconds(1) // intake motor should turn off exactly 0.75 sec into waitSeconds(1)

            .forward(25 + stepIncrement)
            .lineToLinearHeading(new Pose2d(0,-42, Math.toRadians(125)))
            .waitSeconds(1.5)
            .build();

        TrajectorySequence hubToPark = bot.trajectorySequenceBuilder(cycles.end())
            .setReversed(true)
            .splineTo(new Vector2d(-59,-35), Math.toRadians(90))
            .build();

        waitForStart();

        bot.followTrajectorySequence(openingMove);

        for(int i = 0; i < NUM_CYCLES; i++) {
            if (!isStopRequested())
                bot.followTrajectorySequence(cycles);
        }

        bot.followTrajectorySequence(hubToPark);

    }
}
