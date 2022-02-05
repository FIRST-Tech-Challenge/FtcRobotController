package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="driveChain4", group="Linear Opmode")
public class drive4_final extends LinearOpMode {

    //there is no servo code in this program

    public DcMotor motorFrontLeft; //motors declared
    public DcMotor motorBackLeft ;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;

    public void turn() { //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorBackLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorFrontRight.setPower(-gamepad1.right_stick_x * 0.5);
        motorBackRight.setPower(-gamepad1.right_stick_x * 0.5);
    }

    public double angleOfJoystick(double joystickY, double joystickX) { //getting angle of left joystick

        if (joystickY < 0 && joystickX == 0) return 3*3.14159265/2; //back

        if (joystickY >= 0 && joystickX > 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)); //quadrant 1

        if (joystickY > 0 && joystickX < 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265/2; //quadrant 2

        if (joystickY <= 0 && joystickX < 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265); //quadrant 3

        if (joystickY < 0 && joystickX > 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3*3.14159265/2); //quadrant 4

        return 3.14159265/2; //forward

    }

    public void move(double direction) { //move  method
        double turnMoveMagnitude = 1; // larger values means less turning while moving, can be adjusted

        double hypotenuseLeft = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x ) + (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of left motion
        double hypotenuseRight = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of right motion
        double speedFactor = 1;

        if (hypotenuseRight > 1) { //keeps magnitude in bounds just in case
            hypotenuseRight = 1;
        }

        if (hypotenuseLeft > 1) { //keeps magnitude in bounds just in case
            hypotenuseLeft = 1;
        }

        motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuseLeft * speedFactor)); //motor code
        motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuseLeft * speedFactor));
        motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuseRight * speedFactor));
        motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuseRight *speedFactor));
    }

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {  //left stick for movement, right stick for turning

            /*the code below does not send anything to the sensors/record movement yet. */

            if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0) { //movement

                move(angleOfJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x));// move method, gets angle from angleOfJoystick

            } else { //turning on spot

                turn();
            }
            idle();
        }

    }
}