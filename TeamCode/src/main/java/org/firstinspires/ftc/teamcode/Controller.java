package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

     DcMotor armLift = null;

     DcMotor armExtend = null;

     Servo rightGrip = null;

     Servo leftGrip = null;

     Servo Roller = null;
    @Override
    public void runOpMode() {

        //frontLeftMotor = hardwareMap.get(DcMotor.class, "motor0");
        //backLeftMotor = hardwareMap.get(DcMotor.class, "motor1");
        //frontRightMotor = hardwareMap.get(DcMotor.class, "motor2");
        //backRightMotor = hardwareMap.get(DcMotor.class, "motor3");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left_motor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left_motor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right_motor");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right_motor");

        armLift = hardwareMap.dcMotor.get("armLift");
        armExtend = hardwareMap.dcMotor.get("armExtend");

        rightGrip = hardwareMap.servo.get("GripR");
        leftGrip = hardwareMap.servo.get("GripL");
        Roller = hardwareMap.servo.get("Roll");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
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


            if (gamepad2.x) {
                Roller.setPosition(180);
            }

            if (gamepad2.b) {
                Roller.setPosition(0);
            }

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            frontLeftMotor.setPower(leftFrontPower);
            frontRightMotor.setPower(rightFrontPower);
            backLeftMotor.setPower(leftBackPower);
            backRightMotor.setPower(rightBackPower);
            armLift.setPower(gamepad2.left_stick_y);
            armExtend.setPower(gamepad2.right_stick_y);

            Roller.setDirection(Servo.Direction.FORWARD);
            
            Roller.getPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Roller position: ", Roller.getPosition());
            telemetry.update();
        }
    }
}
