package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.cos;
import static java.lang.Math.sin;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.common.Constants;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor arm = null;
    private IMU imu = null;
    private DcMotor intake = null;
    private Servo servo = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back");
        imu = hardwareMap.get(IMU.class, "imu");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");

        leftFrontDrive.setDirection(Constants.motorDirections.get("left_front"));
        leftBackDrive.setDirection(Constants.motorDirections.get("left_back"));
        rightFrontDrive.setDirection(Constants.motorDirections.get("right_front"));
        rightBackDrive.setDirection(Constants.motorDirections.get("right_back"));

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double powercoef = 0.5;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if(gamepad1.right_bumper) powercoef = 1;
            else powercoef = 0.4;
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double forward = -gamepad1.left_stick_y; /* Invert stick Y axis */
            double strafe = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double gyro_radians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double temp = forward * cos(gyro_radians) +
                    strafe * sin(gyro_radians);
            strafe = -forward * sin(gyro_radians) +
                    strafe * cos(gyro_radians);
            forward = temp;

            /* At this point, Joystick X/Y (strafe/forwrd) vectors have been */
            /* rotated by the gyro angle, and can be sent to drive system */

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = powercoef * (forward + strafe + yaw);
            double rightFrontPower = powercoef * (forward - strafe - yaw);
            double leftBackPower = powercoef * (forward - strafe + yaw);
            double rightBackPower = powercoef * (forward + strafe - yaw);

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Other subsystems. Could be refined a lot.
            double desiredArm = gamepad2.right_stick_y;
            arm.setPower(desiredArm/3);

            double desiredIntake = gamepad2.left_stick_y;
            intake.setPower(desiredIntake);
            double desiredSos = gamepad2.left_trigger;
            servo.setPosition(desiredSos);

            // possible setpoint code. assumes position 0 is when arm is horizontal.
            if(gamepad2.right_stick_y < -0.1) arm.setTargetPosition(-Constants.ArmCountsPerRev/24); // intaking
            else if (gamepad2.right_stick_y > 0.1) arm.setTargetPosition(Constants.ArmCountsPerRev/3); //scoring
            else arm.setTargetPosition(0); //carrying
            arm.setPower(1);

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.update();
        }
    }
}
