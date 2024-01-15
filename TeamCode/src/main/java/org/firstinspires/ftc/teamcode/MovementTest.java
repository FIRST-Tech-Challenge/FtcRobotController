package org.firstinspires.ftc.teamcode;

import android.os.Build;

import androidx.annotation.RequiresApi;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MovementTest", group="Linear Opmode")
public class MovementTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor lift_motor = null;


    @RequiresApi(api = Build.VERSION_CODES.N)
    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "0");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "3");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "1");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "2");
        lift_motor = hardwareMap.get(DcMotor.class, "lift_motor");

        DcMotor[] AllMotors = {leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive};

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        for (int i = 0; i < AllMotors.length; i++) {
            AllMotors[i].setPower(0);
        };

        /*
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        */

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            float x = gamepad1.right_stick_x;
            float y = gamepad1.right_stick_y;
            double theta = Math.atan2(x, y);
            float turn = gamepad1.left_stick_x;
            float power = gamepad1.right_stick_x > gamepad1.right_stick_y ? gamepad1.right_stick_x : gamepad1.right_stick_y;
            double sin = Math.sin(theta - Math.PI/4);
            double cos = Math.cos(theta - Math.PI/4);
            double max = Math.abs(cos);

            double leftFrontPower  = cos * power/max + turn;
            double rightFrontPower = sin * power/max + turn;
            double leftBackPower   = sin * power/max + turn;
            double rightBackPower  = cos * power/max + turn;

            telemetry.addData("speed data", "LF: %d\nRF: %d\nLB: %d\nRB: %d");
            telemetry.update();

            if (power + Math.abs(turn) > 1) {
                leftFrontPower  /= power + turn;
                leftBackPower   /= power + turn;
                rightBackPower  /= power + turn;
                rightFrontPower /= power + turn;
            };*/

            float speed = -gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x;
            float strafe = gamepad1.left_stick_x;

            float leftFrontPower = speed + turn + strafe;
            float rightFrontPower = speed - turn - strafe;
            float leftBackPower = speed + turn - strafe;
            float rightBackPower = speed - turn + strafe;


            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            if (gamepad1.dpad_down) {
                lift_motor.setPower(1);
            }
        }
    }
}
