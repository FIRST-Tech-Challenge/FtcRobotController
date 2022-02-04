package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class tapeMeasure extends LinearOpMode {
    private DcMotor altitude = hardwareMap.dcMotor.get("motorFrontLeft");//changes direction up or down
    private DcMotor azimuth = hardwareMap.dcMotor.get("motorFrontLeft");//change direction left or right
    private DcMotor extend = hardwareMap.dcMotor.get("motorFrontLeft");//extend or retract tape measure

    private ElapsedTime runtime;
    public enum states{LOW,MID,HIGH,toLOW,toMID,toHIGH};

    //coefficients for each motor speed. change x or y so that the speed matches with the other.
    double xSpeed = 1;//azimuth speed
    double ySpeed = 1;//altitude speed
    double extendSpeed = 1;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        initialize();
        while (opModeIsActive()) {

            //first control mode: control direction with right joystick and control extension with left joystick's y value. only active while left bumper held down
            // if you decide to use this one, make it so that the drivetrain sets all motor speeds to 0 and doesn't move while leftbumper is held down

            //second control mode: use d-pad for direction, left bumper to retract, left trigger to extend. currently set to this.

            //controlModeA();
            controlModeB();


        }
    }

    public void controlModeA(){
        if (gamepad1.left_bumper) {
            if (Math.pow(gamepad1.right_stick_x, 2) + Math.pow(gamepad1.right_stick_y, 2) >= 0.1) {//controller deadzone is 0.1. just set that number to 0 if you don't need it
                altitude.setPower(-gamepad1.right_stick_y*ySpeed);
                azimuth.setPower(gamepad1.right_stick_x*xSpeed);
            }
            else{//no direction change if right stick within deadzone
                altitude.setPower(0);
                azimuth.setPower(0);
            }

            extend.setPower(-gamepad1.left_stick_y*extendSpeed);
        }
        else{
            altitude.setPower(0);
            azimuth.setPower(0);
            extend.setPower(0);
        }
        return;
    }

    public void controlModeB(){
        if(gamepad1.dpad_up||gamepad1.dpad_down){//altitude
            if(gamepad1.dpad_up) {
                altitude.setPower(1 * ySpeed);
            }
            if(gamepad1.dpad_down){
                altitude.setPower(-1 * ySpeed);
            }
        }
        else{altitude.setPower(0);}

        if(gamepad1.dpad_left||gamepad1.dpad_right){//azimuth
            if(gamepad1.dpad_right) {
                azimuth.setPower(1 * ySpeed);
            }
            if(gamepad1.dpad_left){
                azimuth.setPower(-1 * ySpeed);
            }
        }
        else{azimuth.setPower(0);}


        if(gamepad1.left_trigger!=0){
            extend.setPower(gamepad1.left_trigger*extendSpeed);
        }
        if(gamepad1.left_bumper){
            extend.setPower(-1*extendSpeed);
        }

        return;
    }

    public void initialize(){
        altitude.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        azimuth.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        altitude.setDirection(DcMotorSimple.Direction.REVERSE);// inverted y axis controls; down on joystick makes it point up
        azimuth.setDirection(DcMotorSimple.Direction.FORWARD);
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}