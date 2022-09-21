package org.firstinspires.ftc.teamcode.util;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Basic drive", group="Basic drive")
public class BasicDrive extends LinearOpMode

{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    double max;

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftFront = hardwareMap.get(DcMotor.class, "lf");
        leftBack = hardwareMap.get(DcMotor.class, "lb");
        rightFront = hardwareMap.get(DcMotor.class, "rf");
        rightBack = hardwareMap.get(DcMotor.class, "rb");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        waitForStart();
//        runtime.reset();

        sleep(1000);

        while (opModeIsActive())
        {

            double leftPower;
            double rightPower;
            double leftBackPower;
            double rightBackPower;

            double turn = gamepad1.left_stick_x;
            double drive = -gamepad1.left_stick_y;
            double strafe = gamepad1.right_stick_x;

            leftPower = drive - turn - strafe;
            rightPower = drive + turn + strafe;
            leftBackPower = drive + turn - strafe;
            rightBackPower = drive - turn + strafe;


            if (Math.abs(leftPower) > 1 || Math.abs(leftBackPower) > 1 || Math.abs(rightPower) > 1 || Math.abs(rightBackPower) > 1) {
                max = Math.max(Math.abs(leftPower), Math.abs(leftBackPower));
                max = Math.max(Math.abs(rightPower), max);
                max = Math.max(Math.abs(rightBackPower), max);

                //divide everything by highest power to keep proper ratios of strafe, org.firstinspires.ftc.teamcode.drive, and turn
                leftPower /= 0.5;
                rightPower /= 0.5;
                leftBackPower /= 0.5;
                rightBackPower /= 0.5;
            }

            // Send calculated power to wheels
            leftFront.setPower(leftPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightBackPower);
        }
    }
}
