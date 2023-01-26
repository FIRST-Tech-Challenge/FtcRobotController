package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Mecanum_Drive", group = "TeleOp")

public class Mecanum_Drive extends OpMode {


    DcMotor right_drive;
    DcMotor left_drive;
    DcMotor back_right_drive;
    DcMotor back_left_drive;
    DcMotor Slide;
    ColorSensor color;
    TouchSensor limitSwitch;

    //Claw Mechanism
    Servo ClawX;
    Servo ClawY;



    //Motor Power
    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;
    double ClawPower;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        Slide = hardwareMap.dcMotor.get("Slide");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        ClawX = hardwareMap.servo.get("ClawX");
        ClawY = hardwareMap.servo.get("ClawY");
        color = hardwareMap.get(ColorSensor.class, "color");
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");

        ClawX.setPosition(0.62);
    }


    @Override
    public void loop() {

        //Movement Controller
        right_drivePower = gamepad1.right_stick_y;
        back_left_drivePower = gamepad1.left_stick_y;
        left_drivePower = gamepad1.left_stick_y;
        back_right_drivePower = gamepad1.right_stick_y;


        left_drive.setPower(left_drivePower);
        right_drive.setPower(right_drivePower);
        back_left_drive.setPower(left_drivePower);
        back_right_drive.setPower(right_drivePower);


        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left
        //attachments



        // Precision moves
        if (gamepad1.y) {
            left_drive.setPower(-.23);
            right_drive.setPower(-.23);
            back_left_drive.setPower(-.23);
            back_right_drive.setPower(-.23);
        }

        if (gamepad1.a) {
            left_drive.setPower(.23);
            right_drive.setPower(.23);
            back_left_drive.setPower(.23);
            back_right_drive.setPower(.23);
        }


        if (rightbumper) {

            left_drive.setPower(-1); // left drive is 0
            right_drive.setPower(1); // right drive is 2
            back_left_drive.setPower(1); // back left drive is 1
            back_right_drive.setPower(-1); // back right drive is 3


        } else if (leftbumper) {




            left_drive.setPower(1);
            right_drive.setPower(-1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(1);
        }

        // CLAW ROTATION
        else if (gamepad2.x) {
            ClawY.setPosition(.97);
        }

        else {
            ClawY.setPosition(0.73);
        }
        // OPENING AND CLOSING
        if (gamepad2.b) {
            ClawX.setPosition(0.62);
        }
        if (gamepad2.a) {
            ClawX.setPosition(0.8);
        }

        //INCREASING THE SLIDE'S HEIGHT
        if (gamepad2.right_bumper){
            Slide.setPower(1);
        } else if (limitSwitch.isPressed() & gamepad2.right_bumper){
            Slide.setPower(0.9);
        } else{
            Slide.setPower(0.2);
        }
        // LOWERING AND TURNING OFF THE SLIDE'S POWER
        if (limitSwitch.isPressed() & gamepad2.left_bumper){
            Slide.setPower(0);
        } else if (gamepad2.left_bumper){
            Slide.setPower(-0.75);
        }

       /* if (gamepad2.left_bumper) { //down
            Slide.setPower(-0.4); //(-0.2)
        } else if (gamepad2.right_bumper) { //up
            Slide.setPower(0.9);
        } else {
            if (limitSwitch.isPressed() == false) {
                Slide.setPower(0.2);
            }
        }

        //TOUCH SENSOR CODE
        telemetry.addData("Touch Sensor", limitSwitch.isPressed());
        telemetry.update();

        // If the Magnetic Limit Switch is pressed, stop the motor

         if (gamepad2.left_bumper & limitSwitch.isPressed()){
            Slide.setPower(0);
        } else if (limitSwitch.isPressed() & gamepad2.right_bumper) {
            Slide.setPower(0.9);
        } else if (limitSwitch.isPressed()){ // Otherwise, run the motor
            Slide.setPower(0);
        }*/

        //COLOR SENSOR CODE BELOW

        telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) color).getDistance(DistanceUnit.CM));


        telemetry.addData("red: ", color.red()); //checking for colors
        telemetry.addData("blue: ", color.blue()); //checking for colors
        telemetry.addData("alpha:/light ", color.alpha()); //the amount of light is alpha
        telemetry.update();

        if (((DistanceSensor) color).getDistance(DistanceUnit.CM) <=5) {
            if(color.blue() > 100){
                ClawX.setPosition(0.8);


                telemetry.addData("Claw Pos:", "Closed");
                telemetry.update();

            }
            if(color.red() > 100){
                ClawX.setPosition(0.8);


                telemetry.addData("Claw Pos:", "Closed");
                telemetry.update();


            }
        } else {
            ClawX.setPosition(0.62);
            telemetry.addData("Claw Pos:", "Open");
            telemetry.update();

        }

        telemetry.addData("ClawX Pos", ClawX.getPosition());
        telemetry.addData("ClawY Pos", ClawY.getPosition());
        telemetry.update();
    }
}
