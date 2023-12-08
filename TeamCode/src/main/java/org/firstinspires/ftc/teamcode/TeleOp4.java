package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

//import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * All the code aside from importing, goes within a class file - essentially telling Android Studio-
 * to run this using java
 */
@TeleOp(name = "TeleOp")

public class TeleOp4 extends OpMode {

//*********** Experimental          Mecanums only
//*********** Rev Hub count: 2

    //TETRIX Motors        -recording the motor type as if this file ran autonomous

    private DcMotor motorLeft, motorLeft2,
            motorRight, motorRight2, motorIntake, motorHang;

    private boolean mecanumDriveMode = true, coastMotors = true, hangerPressed = false;
    private float mecanumStrafe = 0, dominantXJoystick = 0;
    private ExtraMotors EM;
    private float maxHang = 1000, droneEnd = 0;
    private double driveMultiplier = 0.75;
    private Servo drone, hangTilt;

    /*
     * Code to run when the op mode is first enabled goes here
     * This is all automatic- it prepares the robot to run loop() without error
     */
    @Override
    public void init() {

//rev hub 1
        motorLeft = hardwareMap.dcMotor.get("front_Left");
        motorRight = hardwareMap.dcMotor.get("front_Right");
        motorLeft2 = hardwareMap.dcMotor.get("back_Left");
        motorRight2 = hardwareMap.dcMotor.get("back_Right");
        motorIntake = hardwareMap.dcMotor.get("Intake");
        motorHang = hardwareMap.dcMotor.get("Hanger");
        hangTilt = hardwareMap.servo.get("HangerUp");
        drone = hardwareMap.servo.get("drone");


        //so you don't have to wire red to black, to maintain program logic
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);
        //telemetry sends data to print onto both phones
        telemetry.addLine("Drive Base TeleOp\nInit Opmode");
    }

    /**
     * The next 2 lines ends the current state of Opmode, and starts Opmode.loop()
     * This enables control of the robot via the gamepad or starts autonomous functions
     */
    @Override
    public void loop() {

        telemetry.addLine("Loop OpMode\ndPadLeft: disable mecanum strafe is " + !mecanumDriveMode +
                "\ndPadRight: enable mecanum strafe is " + mecanumDriveMode +
                "\nX: brake motors is " + !coastMotors + "\nY: coast motors is " + coastMotors);

        //telemetry.addData("LeftMTR  PWR: ", motorLeft.getPower());
        //telemetry.addData("RightMTR PWR: ", motorRight.getPower());
        //telemetry.addData("Arm Height: ", motorArmTilt.getCurrentPosition());
        //telemetry.addData("Arm Extend :", motorIntake.getCurrentPosition());
        //telemetry.addData("Hang Height :", motorHang.getCurrentPosition());
        //telemetry.addData("left trigger :", gamepad2.left_trigger);
        //telemetry.addData("right trigger :", gamepad2.right_trigger);
        telemetry.addData("left_stick_x", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x", gamepad1.right_stick_x);
        telemetry.addData("right_stick_x", gamepad1.right_stick_y);
        telemetry.addData("Servo", drone.getPosition());
        telemetry.addData("tilt: ", hangTilt.getPosition());
        telemetry.addData("extension: ", motorIntake.getCurrentPosition());
telemetry.update();
        //allows for 3-speed joystick control

        if (abs(gamepad1.left_stick_x) > 0.15 || abs(gamepad1.right_stick_x) > 0.15) {
            dominantXJoystick = (abs(gamepad1.left_stick_x) - abs(gamepad1.right_stick_x));
            mecanumDriveMode = true;
        } else {
            mecanumDriveMode = false;
        }

        if (mecanumDriveMode) {     //when enabled, motors will only hit 100% when strafing and driving

            if (dominantXJoystick > 0) {
                mecanumStrafe = gamepad1.left_stick_x;
            } else if (dominantXJoystick < 0) {
                mecanumStrafe = gamepad1.right_stick_x;
            }



                motorLeft.setPower((gamepad1.left_stick_y + mecanumStrafe) * driveMultiplier); // previously 2 * .75
                motorLeft2.setPower((gamepad1.left_stick_y - mecanumStrafe) * driveMultiplier);
                motorRight.setPower((gamepad1.right_stick_y - mecanumStrafe) * driveMultiplier);
                motorRight2.setPower((gamepad1.right_stick_y + mecanumStrafe) * driveMultiplier);



            //button code to manipulate other code/robot
            //if (gamepad1.x) {
                motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                coastMotors = false;
           // } else if (gamepad1.y) {
               // motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
               // motorLeft2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //motorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //motorRight2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                //coastMotors = true;
            //} else if (gamepad1.dpad_left) {
            //    mecanumDriveMode = false;
           // } else if (gamepad1.dpad_right) {
          //      mecanumDriveMode = true;
           // }

//gamepad2

            //problematic as telemetry is changed when gamepad 1 x or y is pressed

        }
        else {
            drive(gamepad1.left_stick_y*0.7, gamepad1.right_stick_y*0.7);
        }
//outtake
        if (gamepad2.left_trigger > 0.25 ) {
            motorIntake.setPower(gamepad2.left_trigger * -0.5);
            //intake
        } else if (gamepad2.right_trigger > 0.25) {
            motorIntake.setPower(gamepad2.right_trigger);
        }
        else{
            motorIntake.setPower(0);
            motorIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (gamepad1.y) {
            motorHang.setPower(1);
        }
        else if(gamepad1.x){
            motorHang.setPower(-1);
        }
        else{
            motorHang.setPower(0);
        }
        if (gamepad1.dpad_up ){
            hangTilt.setPosition(1);
        }
        else if (gamepad1.dpad_down){
            hangTilt.setPosition(0);
        }
        if (gamepad2.a){
            drone.setPosition(1);
        }
        else {
            drone.setPosition(0);
        }




        //else if (gamepad2.y){

       // }
//end of loop opmode programingadb
    }

    @Override
    public void stop() {
        telemetry.clearAll();
        telemetry.addLine("Stopped");
    }

    public void drive(double left, double right) {
        motorLeft.setPower(left);
        motorLeft2.setPower(left);
        motorRight.setPower(right);
        motorRight2.setPower(right);

    }

    private boolean MaxNotReached(DcMotor motor, float value) {
        return motor.getCurrentPosition() < value;
    }

    private boolean MinNotReached(DcMotor motor, float value) {
        return motor.getCurrentPosition() > value;
    }
}