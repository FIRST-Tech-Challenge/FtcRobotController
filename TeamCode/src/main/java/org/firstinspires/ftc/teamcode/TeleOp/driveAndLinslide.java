package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="General Code", group="Linear Opmode")
public class driveAndLinslide extends LinearOpMode {

    public DcMotor LinSlideMotorL, LinSlideMotorR; //motors declared
    public DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH}
    states state = states.LOW;

    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button slightly too long would make it switch heights more than 1 time
    double CDtimer = 0;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2600;

    public void initialize(){//initialize linearSlide and drive motors. it assumes the linear slide starts at the lowest state.
        LinSlideMotorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinSlideMotorL.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        //LinSlideMotorL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LinSlideMotorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LinSlideMotorR.setDirection(DcMotorSimple.Direction.REVERSE);
        //LinSlideMotorR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time
        toggle=0;
    }

    public void desiredSlideHeight(){  //trigger inputs to move slide all the way up or down
  //      if(gamepad1.right_bumper&&(runtime.time()-CDtimer)>=modeCD) {
            if (gamepad1.right_trigger == 1) { //slides to bottom
                state = states.toLOW;

            } else if (gamepad1.right_bumper) {
                state = states.toMID;

            } else if (gamepad1.left_trigger == 1) { //slides to top
                state = states.toHIGH;
            }

        }
  //      CDtimer = runtime.time();   }



    public void moveSlide(){
        switch (state) {
            case LOW:
                if (LinSlideMotorL.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                    state = states.toLOW;

                }
                //code when low goes here
                break;
            case MID:
                if (LinSlideMotorL.getCurrentPosition() != mid) {
                    state = states.toMID;

                }
                break;
            case HIGH:
                if (LinSlideMotorL.getCurrentPosition() != high) {
                    state = states.toHIGH;

                }
                break;

            case toLOW:
                if (LinSlideMotorL.getCurrentPosition() == low) {
                    state = states.LOW;
                } else {

                    LinSlideMotorR.setTargetPosition(low);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(low);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case toMID:
                if (LinSlideMotorL.getCurrentPosition() == mid) {
                    state = states.MID;
                } else {

                    LinSlideMotorR.setTargetPosition(mid);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(mid);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
            case toHIGH:
                if (LinSlideMotorL.getCurrentPosition() == high) {
                    state = states.HIGH;
                } else {

                    LinSlideMotorR.setTargetPosition(high);
                    LinSlideMotorR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    LinSlideMotorL.setTargetPosition(high);
                    LinSlideMotorL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
                break;
        }
    }

    public void turn() { //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorBackLeft.setPower(gamepad1.right_stick_x * 0.5);
        motorFrontRight.setPower(-gamepad1.right_stick_x * 0.5);
        motorBackRight.setPower(-gamepad1.right_stick_x * 0.5);
    }

    public void move(double direction) { //move  method
        double amps = 1; //this is the amplitude of the sin function
        double turnMoveMagnitude = 1; // larger values means less turning while moving, can be adjusted
        double speedFactor = 1; //max speed, between 0 and 1

        double hypotenuseLeft = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x ) + (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of left motion
        double hypotenuseRight = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of right motion

        if (hypotenuseRight > 1) { //keeps magnitude in bounds just in case
            hypotenuseRight = 1;
        }

        if (hypotenuseLeft > 1) { //keeps magnitude in bounds just in case
            hypotenuseLeft = 1;
        }

        // motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * Math.pow(hypotenuseLeft, 2) * speedFactor)); //motor code
        // motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * Math.pow(hypotenuseLeft, 2) * speedFactor));
        // motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * Math.pow(hypotenuseRight, 2) * speedFactor));
        // motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * Math.pow(hypotenuseRight, 2) * speedFactor));

        motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor))); //put the b value as 2 pi and add a linear realtionship to the sin function in order to make it work
        motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor)));
        motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * (amps * Math.sin(hypotenuseRight * 2 * Math.PI) + hypotenuseRight * speedFactor)));
        motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseRight * 2 * Math.PI) + hypotenuseRight * speedFactor)));

        //AN IDEA:
        //I'm thinking that we set some max velocity such as: float MAXvsfrontleft = (Math.sin(direction + (3.14159265 / 4)) * (amps * Math.sin(hypotenuseLeft * 2 * Math.PI) + hypotenuseLeft * speedFactor))
        //and then we have a certain accelartaion such as: double acc = 5;
        //then every time we go through a cycle, we can determine the current velocity from teh accelaration
        //we'll eventually reach the maximum speed through uniform acceleration
        //we can determine the amount of time we need to accelartate for with: t = Maxvsfrontleft/acc;
        //then we simply accelerate for that amount of time
        //we can find the current velocity at each cycle by using uniform accelerated motion equations
        //If we ever move the joystick to another speed, we'll simply set that as a new max and accelerate to that speed.

    }

    public double angleOfJoystick(double joystickY, double joystickX) { //getting angle of left joystick

        if (joystickY < 0 && joystickX == 0) return 3*3.14159265/2; //back

        if (joystickY >= 0 && joystickX > 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)); //quadrant 1

        if (joystickY > 0 && joystickX < 0) return Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265/2; //quadrant 2

        if (joystickY <= 0 && joystickX < 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3.14159265); //quadrant 3

        if (joystickY < 0 && joystickX > 0) return (Math.atan(Math.abs(joystickY)/ Math.abs(joystickX)) + 3*3.14159265/2); //quadrant 4

        return 3.14159265/2; //forward

    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        LinSlideMotorL = hardwareMap.dcMotor.get("LinSlideMotorL");
        LinSlideMotorR = hardwareMap.dcMotor.get("LinSlideMotorR");

        waitForStart();

        if (isStopRequested()) return;

        initialize();
        while (opModeIsActive()) {
            /*the code below does not send anything to the sensors/record movement yet. */

            if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0) { //movement

                move(angleOfJoystick(-gamepad1.left_stick_y, gamepad1.left_stick_x));// move method, gets angle from angleOfJoystick

            } else { //turning on spot

                turn();
            }
            //idle();  this was in drive4. uncomment it if removing it causes issue

//LINSLIDE CODE STARTS HERE

            desiredSlideHeight();

            moveSlide();

            //telemetry
            telemetry.addData("motorPos ", LinSlideMotorL.getCurrentPosition());
            telemetry.addData("motorPos ", LinSlideMotorR.getCurrentPosition());
        }

    }
}
