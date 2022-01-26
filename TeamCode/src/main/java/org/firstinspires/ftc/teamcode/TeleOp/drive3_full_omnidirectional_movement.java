package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="driveChain3", group="Linear Opmode")
public class drive3_full_omnidirectional_movement extends LinearOpMode {

    //there is no servo code in this program

    private DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); //motors declared
    private DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
    private DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
    private DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

    public void turn(){ //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x);
        motorBackLeft.setPower(gamepad1.right_stick_x);
        motorFrontRight.setPower(-gamepad1.right_stick_x);
        motorBackRight.setPower(-gamepad1.right_stick_x);
    }

    public double angleOfJoystick(double joystickY, double joystickX){ //getting angle of left joystick

        double theta;
        if(joystickX==0 && joystickY>0){ //avoids tan90 error

            theta = 3.14159265/2;

        } else if(joystickX==0 && joystickY<0){ //avoids tan270 error

            theta = 3*3.14159265/2;

        } else if(joystickY>0){ //forwards movement

            theta = Math.toRadians(Math.atan(joystickY/joystickX))+(3.14159265/2);

        } else { //this is for backwards movement

            theta = (Math.toRadians(Math.atan(joystickY/joystickX))) + (3*3.14159265/2);

        }

        return theta;
    }
    public void move(double direction){ //main move method
        double hypotenuse = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x); //magnitude of motion

        if(hypotenuse>1){ //keeps magnitude in bounds just in case
            hypotenuse = 1;
        }

        motorFrontLeft.setPower((Math.sin(direction+(3.14159265/4))*hypotenuse)); //motor code
        motorBackLeft.setPower((Math.sin(direction-(3.14159265/4))*hypotenuse));
        motorFrontRight.setPower((Math.sin(direction-(3.14159265/4))*hypotenuse));
        motorBackRight.setPower((Math.sin(direction+(3.14159265/4))*hypotenuse));
    }
    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            /*the code below does not send anything to the sensors/record movement yet. */

            //left stick for movement, right stick for turning

            if(gamepad1.left_stick_y > 0 || gamepad1.left_stick_x > 0) { //movement - has priority

                double trueAngle = angleOfJoystick(gamepad1.left_stick_y, gamepad1.left_stick_x); //gets ideal movement angle from joystick

                move(trueAngle);// main move method

            } else { //turning - low priority. currently can't turn AND move

                turn();

            }
        }
    }

}
