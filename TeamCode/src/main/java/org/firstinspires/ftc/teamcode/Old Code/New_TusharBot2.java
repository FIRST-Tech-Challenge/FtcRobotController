//NOTE: If you are making any changes to controls, please update the list here :)
/*
Controls:

Universal:

Right Trigger : Will slow movement for your controller (NOTE: WILL NOT SLOW BUTTON PRESS ACTIONS ex. A, B, etc.)


Gamepad 1

Left Stick Y : Move forward and back
Left Stick X : Strafe left and right
Right Stick X : Rotate



Gamepad 2

Left Stick Y : Raise/Lower elevator
Right Stick Y : Lift/Lower claw linier Slide
A : Open and close the claw
Y : Flip tray grabbers

*/

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import java.lang.Math;


//Created by Cornelius, Charlie G, and Jackson  :)

@TeleOp(name = "New_TusharBot2", group = "A")
public class New_TusharBot2 extends OpMode {
    // Create motors
    double target = 1;
    //ColorSensor color;
    DcMotor driveFL;
    DcMotor driveFR;
    DcMotor driveBL;
    DcMotor driveBR;

    DcMotor rope;
    public final int maxRope = 6800;
    public final int minRope = 0;

    // Create servos
    Servo claw;
    Servo grabL;
    Servo grabR;
    Servo flipper;

    // Create vars used for input
    boolean dropping = false;

    boolean[] lastButtons = new boolean[20];
    boolean[] buttonPresses = new boolean[20];


    boolean clawClosed = false;
    boolean grab = false;
    double clawClosePos = 0;

    boolean flipDown = false;
    public void init() {

        // Get motors
        driveFL = hardwareMap.dcMotor.get("front left drive");
        driveFR = hardwareMap.dcMotor.get("front right drive");
        driveBL = hardwareMap.dcMotor.get("back left drive");
        driveBR = hardwareMap.dcMotor.get("back right drive");

        // reverse direction of FL and BL motors
        driveFL.setDirection(DcMotorSimple.Direction.REVERSE);
        driveBL.setDirection(DcMotorSimple.Direction.REVERSE);



        // reverse direction of one elevator tilt motor to run both motors in same
        // direction

        // rope = hardwareMap.dcMotor.get("vertical elevator");
        // rope.setDirection(DcMotorSimple.Direction.REVERSE);

        // rope.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rope.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // claw  = hardwareMap.servo.get("claw");
        // grabL = hardwareMap.servo.get("hand servo left");
        // grabR = hardwareMap.servo.get("hand servo right");

        // flipper  = hardwareMap.servo.get("flipper");

        // // run the rope without an encoder
        // rope.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // // set zero power behavior for rope
        // rope.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //  color = hardwareMap.colorSensor.get("color");
        //    color.enableLed(true);

    }


    public void loop() {

        //brake
        float moveSlow = (1 - gamepad1.right_trigger * 0.7f);

        // Drive controls
        float x = gamepad1.left_stick_x;
        float y = -gamepad1.left_stick_y;
        float r = gamepad1.right_stick_x;

        driveBL.setPower((y - x + r) * moveSlow);
        driveFL.setPower((y + x + r) * moveSlow);
        driveBR.setPower((y + x - r) * moveSlow);
        driveFR.setPower((y - x - r) * moveSlow);

        // Tilt elevator

        // telemetry.addData("elevator: ", rope.getCurrentPosition());

        // float cr = color.red();   // Red channel value
        // float cg = color.green(); // Green channel value
        // float cb = color.blue();  // Blue channel value
        //telemetry.addData("red: ", cr);
        //telemetry.addData("green: ", cg);
        //telemetry.addData("blue: ", cb);
        //color.enableLed(!gamepad1.b);


        // Registers when buttons are first pressed on gamepad2 and gamepad1
        boolean[] buttons = new boolean[20];
        buttons[0] = gamepad2.a;
        buttons[1] = gamepad2.y;
        buttons[2] = gamepad2.x;
        buttons[3] = gamepad2.b;
        buttons[4] = gamepad2.dpad_up;
        buttons[5] = gamepad2.dpad_left;
        buttons[6] = gamepad2.dpad_down;
        buttons[7] = gamepad2.dpad_right;
        buttons[8] = gamepad2.left_bumper;
        buttons[9] = gamepad2.right_bumper;

        buttons[10] = gamepad1.a;
        buttons[11] = gamepad1.y;
        buttons[12] = gamepad1.x;
        buttons[13] = gamepad1.b;
        buttons[14] = gamepad1.dpad_up;
        buttons[15] = gamepad1.dpad_left;
        buttons[16] = gamepad1.dpad_down;
        buttons[17] = gamepad1.dpad_right;
        buttons[18] = gamepad1.left_bumper;
        buttons[19] = gamepad1.right_bumper;

        for (int i = 0; i < 20; i++) {
            buttonPresses[i] = (buttons[i] && (!lastButtons[i]));
        }


        double power;



        //Automatically lowers down arm fully
        // if(gamepad2.right_bumper){
        //     dropping = true;
        //     rope.setTargetPosition(minRope);
        //     rope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // }
        // if(dropping){
        //     //Lower arm as fast as possible
        //     power = -1;

        //     if(!rope.isBusy()){
        //         dropping = false;
        //         rope.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //     }
        // }
        // else{
        //     //Choose elevator power based on gamepad stick + trigger
        //     power = -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * 0.7f);
        // }

        // //Limits rope power based on how close it is to limits
        // double maxPower = powerLimit(rope.getCurrentPosition(), maxRope);
        // double minPower = powerLimit(rope.getCurrentPosition(), minRope);
        // if(!gamepad2.left_bumper)
        //     power = clamp(power, minPower, maxPower);
        // rope.setPower(power);

        // telemetry.addData("ropeMaxPower", maxPower);
        // telemetry.addData("ropeMinPower", minPower);
        // telemetry.addData("ropeActualPower", power);

        // telemetry.addData("ropePos",rope.getCurrentPosition());

        // /*




        //Automatically lowers down arm fully
        // if(gamepad2.right_bumper){
        //     dropping = true;
        //     rope.setTargetPosition(minRope);
        //     rope.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        // }
        // if(dropping){
        //     rope.setPower(1);

        //     //if(rope.getCurrentPosition() >= rope.getTargetPosition()){
        //     if(!rope.isBusy()){
        //         dropping = false;
        //         rope.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //     }
        // }
        // else{
        //     //Otherwise control normally with joystick + trigger brake
        //     double power = -gamepad2.left_stick_y * (1 - gamepad2.right_trigger * 0.7f);
        //     // if bumper pressed returns true
        //     if(rope.getCurrentPosition() > maxRope  && !buttons[8]){
        //         rope.setPower(Math.min(0,power));
        //         telemetry.addData("maxed",rope.getCurrentPosition());
        //     }
        //     else if(rope.getCurrentPosition() < minRope){
        //         rope.setPower(Math.max(0,power));
        //         telemetry.addData("minned",rope.getCurrentPosition());
        //     }
        //     else{
        //         rope.setPower(power);
        //     }
        // }


        //     // Open and close claw with A
        //     if (buttonPresses[0]) {
        //         // clawClosePos += .1;
        //         clawClosed = !clawClosed;
        //     }
        //     if(buttonPresses[5]){
        //         target+=0.1;
        //     }
        //     if(buttonPresses[7]){
        //         target-=0.1;
        //     }

        //     // else {
        //     //     clawClosePos -= .1;
        //     //     clawClosePos = max(clawClos);
        //     // }
        //     // claw.setPosition(clawClosePos);

        //     claw.setPosition(clawClosed ? 0 : 0.95); //CHANGE THESE VALUES


        //     telemetry.addData("clawPos", claw.getPosition());


        //     if (buttonPresses[14]) {
        //         // clawClosePos += .1;
        //         flipDown = !flipDown;
        //     }
        //     // else {
        //     //     clawClosePos -= .1;
        //     //     clawClosePos = max(clawClos);
        //     // }
        //     // claw.setPosition(clawClosePos);

        //     flipper.setPosition(flipDown ? 0.2 : 0);
        //     telemetry.addData("flipPose", flipper.getPosition());




        //     // Flip tray grabber things with gamepad1 Y
        //     if (buttonPresses[11]) {
        //         grab = !grab;
        //     }
        //     grabL.setPosition(grab ? 1 : 0);
        //     grabR.setPosition(grab ? 0 : 1);



        //     lastButtons = buttons;

        // //     if (buttonPresses[2]) {
        //         // clawClosePos += .1;
        //       //  flipDown = !flipDown;
        //      //}
        //     //flipper.setPosition(flipDown ? 0.4 : 0.2);
        //      telemetry.update();
        // }


        // public double clamp(double num, double min, double max){
        //     if(num > max){
        //         return max;
        //     }
        //     if(num < min){
        //         return min;
        //     }
        //     return num;
        // }

        // //calculates the power limit based on current position and limit position (max or min pos)
        // public double powerLimit(int currentPos, int limit){
        //     double dist = limit - currentPos;
        //     return (Math.atan(dist/100) * 2/Math.PI);
        // }

    }
}