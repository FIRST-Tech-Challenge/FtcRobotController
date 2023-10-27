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

@TeleOp(name = "MecanumDriveLeague", group = "TeleOp")

public class MecanumDriveLeague extends OpMode {


    //DriveTrain Motors
    private  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL;

    private Servo BucketHold, Bucket;


    //limitswitch
    private TouchSensor LimitSwitch;



    boolean frontDrive = true;

    //Motor Power
    double left_drivePower;
    double right_drivePower;
    double back_right_drivePower;
    double back_left_drivePower;


    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        back_right_drive = hardwareMap.dcMotor.get("back_right_drive");
        back_left_drive = hardwareMap.dcMotor.get("back_left_drive");
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        BucketHold = hardwareMap.get(Servo.class, "BucketHold");





    }


    @Override
    public void loop() {


        while (frontDrive) {
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

            boolean UpSlideBumper = gamepad2.right_bumper;
            boolean DownSlideBumper = gamepad2.left_bumper;


            //attachments

            if(gamepad1.a){
                frontDrive = false;
                break;
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


            if (LimitSwitch.isPressed()) {
                telemetry.addData("Digital Touch", "Pressed");
            } else {
                telemetry.addData("Digital Touch", " Not Pressed");
            }


            if (UpSlideBumper) {
                SlideR.setPower(0.8);
                SlideL.setPower(0.8);
            }
            else{
                SlideR.setPower(0);
                SlideL.setPower(0);
            }

            //Slide Goes Down
            if(!LimitSwitch.isPressed()){
                if(DownSlideBumper){
                    SlideR.setPower(-0.4);
                    SlideL.setPower(-0.4);
                }
            }
            else{
                SlideR.setPower(0);
                SlideL.setPower(0);
            }
        }
        while(!frontDrive){
            right_drivePower = gamepad1.right_stick_y;
            back_left_drivePower = gamepad1.left_stick_y;
            left_drivePower = gamepad1.left_stick_y;
            back_right_drivePower = gamepad1.right_stick_y;


            left_drive.setPower(left_drivePower*-1);
            right_drive.setPower(right_drivePower*-1);
            back_left_drive.setPower(left_drivePower*-1);
            back_right_drive.setPower(right_drivePower*-1);


            boolean rightbumper = gamepad1.right_bumper; //Strafe Right
            boolean leftbumper = gamepad1.left_bumper; //Strafe Left

            boolean UpSlideBumper = gamepad2.right_bumper;
            boolean DownSlideBumper = gamepad2.left_bumper;


            //attachments

            if(gamepad1.a){
                frontDrive = true;
                break;
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


            if (LimitSwitch.isPressed()) {
                telemetry.addData("Digital Touch", "Pressed");
            } else {
                telemetry.addData("Digital Touch", " Not Pressed");
            }


            //Slide Goes Up
            if (UpSlideBumper) {
                SlideR.setPower(0.8);
                SlideL.setPower(0.8);
            }
            else{
                SlideR.setPower(0);
                SlideL.setPower(0);
            }

            //Slide Goes Down
            if(!LimitSwitch.isPressed()){
                if(DownSlideBumper){
                    SlideR.setPower(-0.4);
                    SlideL.setPower(-0.4);
                }
            }
            else{
                SlideR.setPower(0);
                SlideL.setPower(0);
            }
        }







    /*


        // Precision moves
        if (gamepad1.y) { //forward
            left_drive.setPower(-.5);
            right_drive.setPower(-.5);
            back_left_drive.setPower(-.5);
            back_right_drive.setPower(-.5);
        }

        if (gamepad1.a) { //back
            left_drive.setPower(.5);
            right_drive.setPower(.5);
            back_left_drive.setPower(.5);
            back_right_drive.setPower(.5);
        }
        if (gamepad1.x) {
            left_drive.setPower(0.42);
            right_drive.setPower(-0.42);
            back_left_drive.setPower(0.42);
            back_right_drive.setPower(-0.42);
        }

        //hi
        if (gamepad1.b) {
            left_drive.setPower(-0.42);
            right_drive.setPower(0.42);
            back_left_drive.setPower(-0.42);
            back_right_drive.setPower(0.42);
        }
     */
    }
}
