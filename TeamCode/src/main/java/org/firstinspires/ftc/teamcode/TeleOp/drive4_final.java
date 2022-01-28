package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="driveChain4", group="Linear Opmode")
public class drive4_final extends LinearOpMode {

    //there is no servo code in this program

    private DcMotor motorFrontLeft; //motors declared
    private DcMotor motorBackLeft ;
    private DcMotor motorFrontRight;
    private DcMotor motorBackRight ;

    public void turn() { //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x);
        motorBackLeft.setPower(gamepad1.right_stick_x);
        motorFrontRight.setPower(-gamepad1.right_stick_x);
        motorBackRight.setPower(-gamepad1.right_stick_x);
    }

    public double angleOfJoystick(double joystickY, double joystickX) { //getting angle of left joystick

        if (joystickY < 0 && joystickX == 0) return 3*3.14159265/2;

        if (joystickY == 0 && joystickX <= 0) return 3.14159265;

        if (joystickY >= 0 && joystickX > 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX));

        if (joystickY >= 0 && joystickX < 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265/2;

        if (joystickY <= 0 && joystickX < 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265);

        if (joystickY <= 0 && joystickX > 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3*3.14159265/2);

        return 3.14159265/2;

    }

    public void move(double direction) { //main move method
        double hypotenuse = Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x); //magnitude of motion

        if (hypotenuse > 1) { //keeps magnitude in bounds just in case
            hypotenuse = 1;
        }

        motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuse)); //motor code
        motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuse));
        motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuse));
        motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuse));
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
                double trueAngle = angleOfJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                System.out.println(trueAngle);
                move(trueAngle);// main move method, gets angle from angleOfJoystic(k)

            } else { //turning. currently can't turn AND move

                turn();
            }
            idle();
        }

    }
}