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
import org.firstinspires.ftc.teamcode.common.Button;

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
    private Button intakeButton = new Button();
    double powercoef = 0.5;
    enum scoringPosition {
        INTAKE,
        LOW,
        MID,
        HIGH
    }

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
        while(!isStarted() && !isStopRequested()) {
            telemetry.addData("Arm position: ", arm.getCurrentPosition());
            telemetry.addData("Arm joystick position: ", gamepad2.right_stick_y);
        }

        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // arm.setPower(gamepad2.right_stick_y * 1/4);
            // servo.setPosition(position);
            // intake.setPower(-gamepad2.right_trigger);
            handleArm();
            // 0.3 is flat, 0.67 is 90 degrees.

            HandleDrivetrain();
            if(gamepad2.y) servo.setPosition(0.1);
            telemetry.update();
        }
    }


    /*

    if trigger held set arm position to -33 and servo position to 0.84
    if joystick up and position between than -33 and 0, set servo to 0.95
    when position greater than 0 set servo to 0.15
    when scoring set position to 0.7
    */

    public void handleArm(){
        double servoSetpoint = 0;
        /*
        if(intakeButton.is(Button.State.TAP)) {
            servoSetpoint = 0.84;
            arm.setTargetPosition(-33);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.3);
            intake.setPower(1);
        } else if (intakeButton.is(Button.State.OFF)) {
            servoSetpoint = 0.95;
            arm.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.3);
            intake.setPower(0);
        } else if (gamepad2.x) {
            servoSetpoint = 0.7;
            arm.setPower(gamepad2.right_stick_y * 1/4);
        } else {
            servoSetpoint = 0.15;
            arm.setPower(gamepad2.right_stick_y * 1/4);
        }

         */
        telemetry.addData("Trigger: ", gamepad2.right_trigger);
        intakeButton.update(gamepad2.right_trigger > 0.5);
        telemetry.addData("Arm position: ", arm.getCurrentPosition());
        telemetry.addData("Arm joystick position: ", gamepad2.right_stick_y);

        if(intakeButton.is(Button.State.TAP)) {
            // Intaking- make sure the pan doesn't have to clear the intake because the pan will be lowered to ground level.
            telemetry.addLine("Beginning intaking");
            // servoSetpoint = 0.84;
            servo.setPosition(0.84);
            arm.setTargetPosition(-207);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(0.25);
            intake.setPower(-1);
            while (arm.isBusy()) telemetry.addLine("Arm moving");
        } else if(intakeButton.is(Button.State.HELD)) {
            servoSetpoint = 0.84;
        } else if (intakeButton.is(Button.State.UP)){
            telemetry.addLine("Finishing intaking");
            // We just finished intaking, so set motors to how they should be
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.setPower(0);
        } else if (gamepad2.x) {
            telemetry.addLine("Scoring");
            // Scoring
            servoSetpoint = 0.7;
            arm.setPower(gamepad2.right_stick_y * 1/4);
        } else if (arm.getCurrentPosition() > 0 && gamepad2.right_stick_y > 0) {
            servoSetpoint = 0.15;
            arm.setPower(gamepad2.right_stick_y * 1 / 4 + 0.01);
        } else if (arm.getCurrentPosition() < 400) { // if it's negative we're going down
            telemetry.addLine("Stowing/intake clearing mode- downward");
            // Stowed position that can also clear the intake. TODO: tune feedforward
            servoSetpoint = 0.95;
            arm.setPower(gamepad2.right_stick_y * 1/4 + 0.002);
        /*
        } else if (arm.getCurrentPosition() < 0) {
            telemetry.addLine("Stowing/intake clearing mode- nominal/upward");
            servoSetpoint = 0.95;
            arm.setPower(gamepad2.right_stick_y * 1 / 4 + 0.01);

         */
        } else {
            telemetry.addLine("Scoring mode");
            // Scoring mode- getting ready to score. The servo will tip to hold the pixel when the
            // arm is out, so move the arm quickly.
            servoSetpoint = 0.15;
            arm.setPower (gamepad2.right_stick_y * 1/4);
        }

        telemetry.addData("Servo setpoint: ", servoSetpoint);
        telemetry.addData("Servo position: ", servo.getPosition());

        if(servoSetpoint != servo.getPosition()) servo.setPosition(servoSetpoint);
    }

    public void HandleDrivetrain() {
        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        if(gamepad1.right_bumper) powercoef = 1;
        else powercoef = 0.4;

        double forward = -gamepad1.left_stick_y; /* Invert stick Y axis */
        double strafe = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double gyro_radians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        telemetry.addData("gyro angle: ", gyro_radians);
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

    }
}
