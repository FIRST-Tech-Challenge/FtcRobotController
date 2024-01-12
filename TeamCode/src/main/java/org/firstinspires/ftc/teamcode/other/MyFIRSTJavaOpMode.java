package org.firstinspires.ftc.teamcode.other;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Me")
@Disabled
public class MyFIRSTJavaOpMode extends LinearOpMode {


    private DcMotor armRotate;
    private DcMotor armBrace;
    private DcMotor armExt;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;


    /**
     * This function is executed when this OpMode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {

        //Initialize Variables
        //Tracks rotation of the arm's motor
        int rotation;
        //Tracks the extension of the arm
        int ext;
        //X and Y values of stick inputs to compile drive outputs
        double yI;
        double xI;
        //Value for armLock
        boolean armLocked;

        armRotate = hardwareMap.get(DcMotor.class, "armRotate");
        armBrace = hardwareMap.get(DcMotor.class, "armBrace");
        armExt = hardwareMap.get(DcMotor.class, "armExt");
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        //Sets initial positions and directions for grip servos
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        //Sets variables to 0 on initialization
        rotation = 0;
        ext = 0;
        armLocked = false;
        waitForStart();

        if (opModeIsActive()) {

            //Sets behaviors and modes for motors
            //ArmExtension and ArmRotate are set to brake when receiving zero power
            //Arm Extension is set to run using encoder outputs and inputs
            armRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armExt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armExt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            armBrace.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armBrace.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

            while (opModeIsActive()) {

                //Assigns variables to inputs
                //Ext and Rotation are set to receive inputs from encoders
                ext = armExt.getCurrentPosition();
                rotation = armRotate.getCurrentPosition();
                //Multipliers are applied to X and Y and they are tied to sticks on the gamepads
                yI = gamepad1.left_stick_y;
                xI = gamepad1.right_stick_x;

                //Sets power for driving wheels
                //low power mode
                if(gamepad1.right_bumper){
                    if(yI > 0.2 || yI < -0.2){
                        backLeftMotor.setPower(((yI-0.1)*-1)*0.25);
                        backRightMotor.setPower(yI*0.25);
                        frontLeftMotor.setPower(((yI-0.1)*-1)*0.25);
                        frontRightMotor.setPower(yI*0.25);
                    } else if (xI > 0.2) {
                        backLeftMotor.setPower((xI-0.1)*0.25);
                        backRightMotor.setPower(xI*0.25);
                        frontLeftMotor.setPower((xI-0.1)*0.25);
                        frontRightMotor.setPower(xI*0.25);
                    } else if (xI < -0.2) {
                        backLeftMotor.setPower((xI-0.1)*0.25);
                        backRightMotor.setPower(xI*0.25);
                        frontLeftMotor.setPower((xI-0.1)*0.25);
                        frontRightMotor.setPower(xI*0.25);
                    } else {
                        backLeftMotor.setPower(0);
                        backRightMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                    }
                }else {
                    //normal driving
                    backLeftMotor.setPower((-yI+xI)*0.9);
                    backRightMotor.setPower(yI+xI);
                    frontLeftMotor.setPower((-yI+xI)*0.9);
                    frontRightMotor.setPower(yI+xI);
                } //end drive controls

                //Arm rotation controls
                //Rotates up when Right Bumper is pressed
                //Rotates down when Left Bumper is pressed
                //Otherwise power is set to 0 (BRAKE)

                //if(armLocked == false){

                if(armLocked == true){
                    armRotate.setPower(0.05);
                    armBrace.setPower(0.05);
                } else{
                    if (gamepad2.right_bumper) {
                        armRotate.setPower(0.3);
                        armBrace.setPower(0.3);
                    } else if (gamepad2.left_bumper) {
                        armRotate.setPower(-0.2);
                        armBrace.setPower(-0.2);
                    } else {
                        armRotate.setPower(0);
                        armBrace.setPower(0);
                    }
                }//end arm rotation inputs




       /*if(armLocked == false){
         if(gamepad2.y){
           armLocked = true;
         }
        } else if (armLocked == true){
          if(gamepad2.x){
            armLocked = false;
          }
        }*/

                //Arm extension controls
                //Moves up when X is pressed
                //Moves down when B is pressed
                //Otherwise set power to 0 (BRAKE)
                //Limiters are applied if the motors position is less than 350° or greater than 2900°
                //If the position is less than 350°, the arm can only extend forward
                //If the position is greater than 2900°, the arm can only retract
                if (ext > 350 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext < 350 && ext < 2900) {
                    if (gamepad2.dpad_up) {
                        armExt.setPower(1);
                    } else {
                        armExt.setPower(0);
                    }
                } else if (ext > 2900 && ext > 350) {
                    if (gamepad2.dpad_down) {
                        armExt.setPower(-1);
                    } else {
                        armExt.setPower(0);
                    }
                } if(gamepad2.right_trigger > 0.5 && gamepad2.left_trigger > 0.5){
                    armExt.setPower(-1);
                    sleep(1000000);
                } //end arm extension inputs



                //Telemetry for debugging
                telemetry.addData("Current Arm Extension", ext);
                telemetry.addData("Current Arm Rotation", armRotate.getCurrentPosition());
                telemetry.addData("Arm lift Power", armRotate.getPower());
                telemetry.addData("Right Wheel Power", backRightMotor.getPower());
                telemetry.addData("Left Wheel Power", backLeftMotor.getPower());
                telemetry.addData("Arm Locked: ", armLocked);
                telemetry.update();
            } //end while loop
        } //end if loop
    } //end run method
} //end class
