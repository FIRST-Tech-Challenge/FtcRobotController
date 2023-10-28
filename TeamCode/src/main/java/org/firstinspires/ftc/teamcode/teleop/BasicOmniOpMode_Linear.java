package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.copySign;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
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
    private DcMotorEx arm = null;
    private IMU imu = null;
    private DcMotor intake = null;
    private Servo servo = null;
    double powercoef = 0.5;

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

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Wait for the game to start (driver presses PLAY)
        double position = 0;
        double armposition = 0;
        boolean prev_bumper;
        boolean two_prev_bumper;
        while(!isStarted() && !isStopRequested()) {
        }

        runtime.reset();

        double armSetpoint = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // setpoint code. assumes arm position 0 is when arm is horizontal, and servo angle increases when angle between top of arm and top of pan decreases
            /*
                                   _____
           armSetPoint, in degrees ->◟/| (this angle is negative)
                                     / |
                                    /  |
                                ___/   |
         servo pos, in degrees ->◟/    | (this angle is positive)
                                 /     |
             */

            // Calculate arm setpoint
            /*
            // safety limits
            if (armSetpoint < -15 && -gamepad2.right_stick_y < 0 ||
                armSetpoint > 120 && -gamepad2.right_stick_y > 0) { // we're up to no good, cautiously proceed
                armSetpoint += gamepad2.right_stick_y/20;
            } else {
                armSetpoint += gamepad2.right_stick_y/200;
            }
             */

            /* fancier controls
            // Score, intake, or raise arm to setpoint
            if(gamepad2.right_trigger > 0.5) {
                arm.setTargetPosition((int) (-15 * Constants.ArmCountsPerDegree)); // intaking
                intake.setPower(1);
                servo.setPosition(Constants.NormalDegreesToServoUnits(15));
            }
            else if (gamepad2.x) servo.setPosition(Constants.NormalDegreesToServoUnits(15)); // scoring
            else {
                arm.setTargetPosition((int) armSetpoint*Constants.ArmCountsPerRev);
                servo.setPosition(-Constants.NormalDegreesToServoUnits(armSetpoint));
            }
             */

            position += gamepad2.left_stick_y/200;
            arm.setPower(gamepad2.right_stick_y * 1/4);
            // servo.setPosition(position);
            intake.setPower(-gamepad2.right_trigger);
            // 0.3 is flat, 0.67 is 90 degrees.
            telemetry.addData("servo target position: ", position);
            telemetry.addData("arm setpoint: ", armposition);
            telemetry.addData("Arm position: ", arm.getCurrentPosition()/Constants.ArmCountsPerDegree);
            servo.setPosition(position);
            telemetry.update();

            HandleDrivetrain();

            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Servo position: ", servo.getPosition());
            telemetry.update();
        }
    }

    public void HandleDrivetrain() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        if(gamepad1.right_bumper) powercoef = 1;
        else powercoef = 0.4;

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
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
    }
}
