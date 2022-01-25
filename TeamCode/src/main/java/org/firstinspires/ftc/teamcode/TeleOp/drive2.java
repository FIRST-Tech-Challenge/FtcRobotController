package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name="driveChain2", group="Linear Opmode")
public class drive2 extends LinearOpMode {

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

            theta = Math.toRadians(Math.tan(joystickY/joystickX));

        } else { //this is for backwards movement

            theta = (Math.toRadians(Math.tan(joystickY/joystickX))) + 3.14159265;

        }

        return theta;
    }

    public double sidewaysForce(double joystickY, double joystickX){
        return (Math.sqrt((joystickX*joystickX)+(joystickY*joystickY)))/1.41421;
    }

    public void move (double nearest45, double diagonalForce){ //movement method
        //using double to allow modification to trueAngle in the future



        if(nearest45 < 0.1){ // move right
            motorFrontLeft.setPower(gamepad1.left_stick_x);
            motorBackLeft.setPower(-gamepad1.left_stick_x);
            motorFrontRight.setPower(-gamepad1.left_stick_x);
            motorBackRight.setPower(gamepad1.left_stick_x);

        }else if(nearest45 < (3.14159265/4)+0.3){ // move front-right
            motorFrontLeft.setPower(diagonalForce);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(diagonalForce);

        }else if(nearest45 < (2*(3.14159265/4))+0.3){ // move front
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorFrontRight.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.left_stick_y);

        }else if(nearest45 < (3*(3.14159265/4))+0.3){ // move front-left
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(diagonalForce);
            motorFrontRight.setPower(diagonalForce);
            motorBackRight.setPower(0);

        }else if(nearest45 < (4*(3.14159265/4))+0.3){ // move left
            motorFrontLeft.setPower(gamepad1.left_stick_x);
            motorBackLeft.setPower(-gamepad1.left_stick_x);
            motorFrontRight.setPower(-gamepad1.left_stick_x);
            motorBackRight.setPower(gamepad1.left_stick_x);

        }else if(nearest45 < (5*(3.14159265/4))+0.3){ // move back-left
            motorFrontLeft.setPower(0);
            motorBackLeft.setPower(-diagonalForce);
            motorFrontRight.setPower(-diagonalForce);
            motorBackRight.setPower(0);

        }else if(nearest45 < (6*(3.14159265/4))+0.3){ // move back
            motorFrontLeft.setPower(gamepad1.left_stick_y);
            motorBackLeft.setPower(gamepad1.left_stick_y);
            motorFrontRight.setPower(gamepad1.left_stick_y);
            motorBackRight.setPower(gamepad1.left_stick_y);

        }else if(nearest45 < (7*(3.14159265/4))+0.3){ // move back-right
            motorFrontLeft.setPower(-diagonalForce);
            motorBackLeft.setPower(0);
            motorFrontRight.setPower(0);
            motorBackRight.setPower(-diagonalForce);

        }else { // move right
            motorFrontLeft.setPower(gamepad1.left_stick_x);
            motorBackLeft.setPower(-gamepad1.left_stick_x);
            motorFrontRight.setPower(-gamepad1.left_stick_x);
            motorBackRight.setPower(gamepad1.left_stick_x);

        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()){

            /*currently, there is no code for moving at angles other than 90 and 45 deg.
            Planning to add that after I understand how the math behind that works. Also,
            the code below does not send anything to the sensors/record movement yet. */

            //left stick for movement, right stick for turning

            if(gamepad1.left_stick_y > 0 || gamepad1.left_stick_x > 0) { //movement - has priority

                double trueAngle = angleOfJoystick(gamepad1.left_stick_y, gamepad1.left_stick_x); //gets ideal movement angle
                double roundedAngle = (Math.round(trueAngle/(3.14159265/4)))*(3.14159265/4);


                move(roundedAngle, sidewaysForce(gamepad1.left_stick_y,gamepad1.left_stick_x)); //main move method

            } else { //turning - low priority. currently can't turn AND move

                turn();

            }
        }
    }

}
