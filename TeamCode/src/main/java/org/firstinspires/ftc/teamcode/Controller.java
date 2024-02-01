package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "controller movement", group = "SA_FTC")
public class Controller extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    ElapsedTime runtime = new ElapsedTime();
    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;

    DcMotorEx armLift = null;

    DcMotor armExtend = null;

    Servo rightGrip = null;

    Servo leftGrip = null;

    Servo roller = null;

    int currentArmPosition = 0;

    public void Movement() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.right_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.right_stick_x;
        double yaw = gamepad1.left_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

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

        // Send calculated power to wheels
        frontLeftMotor.setPower(leftFrontPower);
        frontRightMotor.setPower(rightFrontPower);
        backLeftMotor.setPower(leftBackPower);
        backRightMotor.setPower(rightBackPower);
    }

    @Override
    public void runOpMode() {

        frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        backLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        //frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        //backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        //frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        //backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        armLift = (DcMotorEx) hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");
        rightGrip = hardwareMap.servo.get("GripR");
        leftGrip = hardwareMap.servo.get("GripL");
        roller = hardwareMap.servo.get("Roll");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        roller.setDirection(Servo.Direction.FORWARD);
        armExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        rightGrip.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // arm test
        armLift.setDirection(DcMotor.Direction.REVERSE);
        
        armLift.setTargetPosition(0);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        armLift.setTargetPosition(300);
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        waitForStart();
        runtime.reset();

        armLift.setVelocity(100);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Movement
            Movement();

            // Roller
            if (gamepad2.x) {
                roller.setPosition(0.275);
            }

            if (gamepad2.b) {
                roller.setPosition(0.95);
            }

            // Arm
            armExtend.setPower(gamepad2.right_stick_y);
            
            // testing debugging
            telemetry.addData("current position", armLift.getCurrentPosition());
            telemetry.addData("target position", armLift.getTargetPosition());
            telemetry.addData("Left stick y", (int)gamepad2.left_stick_y);
            telemetry.addData("currentarmposition", currentArmPosition);

            // Grip
            if (gamepad2.left_bumper) {
                leftGrip.setPosition(0.4);
            }

            if (gamepad2.right_bumper) {
                rightGrip.setPosition(0.4);
            }

            if (gamepad2.left_trigger > 0.01) {
                leftGrip.setPosition(0);
            }

            if (gamepad2.right_trigger > 0.01) {
                rightGrip.setPosition(0);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            // telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            // telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            // telemetry.addData("roller position: ", roller.getPosition());
            telemetry.update();
        }
    }
}
