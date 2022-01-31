package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Control+Cascade", group="Linear Opmode")
public class driveAndLinslide extends LinearOpMode {

    private DcMotor motor = hardwareMap.dcMotor.get("motorFrontLeft");//hardware
    public DcMotor motorFrontLeft; //motors declared
    public DcMotor motorBackLeft ;
    public DcMotor motorFrontRight;
    public DcMotor motorBackRight;
    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};
    states state = states.LOW;

    private int toggle;//toggle for setting height
    final double modeCD = 0.15;//these two values are for putting a cooldown on switching heights, just in case pushing down the button slightly too long would make it switch heights more than 1 time
    double CDtimer = 0;

    //Encoder positions for each level on linear slide
    final int low = 0;
    final int mid = 1200;
    final int high = 2600;

    public void initialize(){//initialize linearSlide. it assumes the linear slide starts at the lowest state.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotorSimple.Direction.FORWARD);//change it if needed
        runtime = new ElapsedTime(ElapsedTime.Resolution.SECONDS);//gets time
        toggle=0;
    }

    public void turn() { //turning method
        motorFrontLeft.setPower(gamepad1.right_stick_x);
        motorBackLeft.setPower(gamepad1.right_stick_x);
        motorFrontRight.setPower(-gamepad1.right_stick_x);
        motorBackRight.setPower(-gamepad1.right_stick_x);
    }

    public void move(double direction) { //move  method
        double turnMoveMagnitude = 2; // larger values means less turning while moving, can be adjusted

        double hypotenuseLeft = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x ) + (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of left motion
        double hypotenuseRight = (Math.hypot(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (gamepad1.right_stick_x/turnMoveMagnitude)) / (1+(Math.ceil(Math.abs(gamepad1.right_stick_x))/turnMoveMagnitude)); //magnitude of right motion

        if (hypotenuseRight > 1) { //keeps magnitude in bounds just in case
            hypotenuseRight = 1;
        }

        if (hypotenuseLeft > 1) { //keeps magnitude in bounds just in case
            hypotenuseLeft = 1;
        }

        motorFrontLeft.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuseLeft)); //motor code
        motorBackLeft.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuseLeft));
        motorFrontRight.setPower((Math.sin(direction - (3.14159265 / 4)) * hypotenuseRight));
        motorBackRight.setPower((Math.sin(direction + (3.14159265 / 4)) * hypotenuseRight));
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
        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


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
            //idle();  this was in drive4. un comment it if removing it causes issue

//LINSLIDE CODE STARTS HERE
            if(gamepad1.right_bumper&&(runtime.time()-CDtimer)>=modeCD){
                if(toggle==2){
                    toggle=-1;
                }
                toggle+=1;
                switch(toggle){
                    case 0:
                        state= states.toLOW;
                        break;
                    case 1:
                        state= states.toMID;
                        break;
                    case 2:
                        state = states.toHIGH;
                        break;
                }
                CDtimer=runtime.time();
            }
            if(gamepad1.right_trigger==1){
                state= states.toLOW;
            }

            switch (state) {
                case LOW:
                    if (motor.getCurrentPosition() != low) {//checks position again to see if overshoot when toLOW ended. state MID and HIGH do the same.
                        state = states.toLOW;
                        break;
                    }
                    //code when low goes here
                    break;
                case MID:
                    if (motor.getCurrentPosition() != mid) {
                        state = states.toMID;
                        break;
                    }
                    break;
                case HIGH:
                    if (motor.getCurrentPosition() != high) {
                        state = states.toHIGH;
                        break;
                    }
                    break;

                case toLOW:
                    if (motor.getCurrentPosition() == low) {
                        state = states.LOW;
                    } else {
                        //motor.setPower(PID(low,prevPos,prevTime));
                        motor.setTargetPosition(low);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
                case toMID:
                    if (motor.getCurrentPosition() == mid) {
                        state = states.MID;
                    } else {
                        //motor.setPower(PID(mid,prevPos,prevTime));
                        motor.setTargetPosition(mid);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
                case toHIGH:
                    if (motor.getCurrentPosition() == high) {
                        state = states.HIGH;
                    } else {
                        //motor.setPower(PID(high,prevPos,prevTime));
                        motor.setTargetPosition(high);
                        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    }
                    break;
            }

            //telemetry
            telemetry.addData("motorPos ", motor.getCurrentPosition());
        }

    }
}
