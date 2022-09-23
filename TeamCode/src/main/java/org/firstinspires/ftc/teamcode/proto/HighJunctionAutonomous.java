package org.firstinspires.ftc.teamcode.proto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "HighJunctionAutonomous")
public class HighJunctionAutonomous extends LinearOpMode
{
    // Initialize motors
    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;

    static final double     COUNTS_PER_MOTOR_GOBUILDA202    = 384.5 ;    // eg: TETRIX Motor Encoder REV    = 1440
    static final double     DRIVE_GEAR_REDUCTION    = 3.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_GOBUILDA202 * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private int leftFrontPos;
    private int rightFrontPos;
    private int leftBackPos;
    private int rightBackPos;

    //Four modes
    //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    @Override
    public void runOpMode() throws InterruptedException
    {
        leftFront = hardwareMap.get(DcMotor.class, "lf");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        leftFrontPos = 0;
        rightFrontPos = 0;
        leftBackPos = 0;
        rightBackPos = 0;

        waitForStart();

//        drive(COUNTS_PER_INCH * 25, COUNTS_PER_INCH * 25, COUNTS_PER_INCH * 25, COUNTS_PER_INCH * 25, 100);
//        Thread.sleep(6000);
//        drive(COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, COUNTS_PER_INCH * 8, 0.5);
        Thread.sleep(100);
        drive(-COUNTS_PER_INCH * 40, COUNTS_PER_INCH * 40, -COUNTS_PER_INCH * 40, COUNTS_PER_INCH * 40, 0.25);
        // 1500 is about 90 turns degrees
        Thread.sleep(100);
//        drive(COUNTS_PER_INCH * 32, COUNTS_PER_INCH * 32, COUNTS_PER_INCH * 32, COUNTS_PER_INCH * 32, 100);
        // forward drive(1000, 1000, 1000, 1000, 0.25);
        // Turn drive(1000, -1000, 1000, -1000, 0.25);
        // strafe drive(1000, -1000, -1000, 1000, 0.25);
    }

    private void drive(double leftFrontTarget, double rightFrontTarget, double leftBackTarget, double rightBackTarget, double speed)
    {
        leftFrontPos += leftFrontTarget;
        rightFrontPos += rightFrontTarget;
        leftBackPos += leftBackTarget;
        rightBackPos += rightBackTarget;

        leftFront.setTargetPosition(leftFrontPos);
        rightFront.setTargetPosition(rightFrontPos);
        leftBack.setTargetPosition(leftBackPos);
        rightBack.setTargetPosition(rightBackPos);

        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(speed);
        rightFront.setPower(speed);
        leftBack.setPower(speed);
        rightBack.setPower(speed);

        while (opModeIsActive() && leftFront.isBusy() && rightFront.isBusy() && leftBack.isBusy() && rightBack.isBusy())
        {
            idle();
        }

    }
}
