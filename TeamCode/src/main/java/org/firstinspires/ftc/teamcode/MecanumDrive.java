package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "MecanumDrive", group = "TeleOp")

public class MecanumDrive extends OpMode {


    //DriveTrain Motors
    private  DcMotor right_drive, left_drive, back_right_drive, back_left_drive;
    //Slide Motors
    private DcMotor SlideR, SlideL, Intake;

    private Servo BucketHold, Bucket;
    private Servo Drone;

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
        left_drive = hardwareMap.dcMotor.get("lm");
        right_drive = hardwareMap.dcMotor.get("rm");
        back_right_drive = hardwareMap.dcMotor.get("brm");
        back_left_drive = hardwareMap.dcMotor.get("blm");
        SlideR = hardwareMap.dcMotor.get("SlideR");
        SlideL = hardwareMap.dcMotor.get("SlideL");
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        back_right_drive.setDirection(DcMotor.Direction.REVERSE);
        LimitSwitch = hardwareMap.get(TouchSensor.class, "LimitSwitch");
        Bucket = hardwareMap.get(Servo.class, "Bucket");
        Drone = hardwareMap.get(Servo.class, "Drone");
        Intake = hardwareMap.dcMotor.get("Intake");






    }


    @Override
    public void loop() {



        //Movement Controller
        right_drivePower = gamepad1.right_stick_y *-1 ;
        back_left_drivePower = gamepad1.left_stick_y;
        left_drivePower = gamepad1.left_stick_y;
        back_right_drivePower = gamepad1.right_stick_y * -1;


        left_drive.setPower(left_drivePower);
        right_drive.setPower(right_drivePower);
        back_left_drive.setPower(left_drivePower);
        back_right_drive.setPower(right_drivePower);


        boolean rightbumper = gamepad1.right_bumper; //Strafe Right
        boolean leftbumper = gamepad1.left_bumper; //Strafe Left

        boolean UpSlideBumper = gamepad2.right_bumper;
        boolean DownSlideBumper = gamepad2.left_bumper;


        //attachments

        if (gamepad2.x){
            Bucket.setPosition(0.2);
        }
        if(gamepad2.dpad_down){
            Bucket.setPosition(0.7);
        }
        if(gamepad2.dpad_up){
            Bucket.setPosition(0);
        }
        if (rightbumper) {

            left_drive.setPower(-1); // left drive is 0
            right_drive.setPower(-1); // right drive is 2
            back_left_drive.setPower(1); // back left drive is 1
            back_right_drive.setPower(1); // back right drive is 3


        } else if (leftbumper) {


            left_drive.setPower(1);
            right_drive.setPower(1);
            back_left_drive.setPower(-1);
            back_right_drive.setPower(-1);
        }


        // CLAW ROTATION


        if (LimitSwitch.isPressed()) {
            telemetry.addData("Digital Touch", "Pressed");
        } else {
            telemetry.addData("Digital Touch", " Not Pressed");
        }


        if (UpSlideBumper) {
            //BucketHold.setPosition(0);
            Bucket.setPosition(0.7);
            SlideR.setPower(0.5);
            SlideL.setPower(0.5);

        }
        else{
            SlideR.setPower(0.1);
            SlideL.setPower(0.1);
        }
        if(gamepad2.b){
            Bucket.setPosition(1);
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




        //intake
        if(gamepad2.a) {
            Intake.setPower(1);
            telemetry.addData("Motor", "1 power");
        }
      /*  else if(gamepad2.b){
            Intake.setPower(-1);
            telemetry.addData("Motor", "-1 Power");
        } */

        else Intake.setPower(0);

        if (gamepad2.dpad_down) {
            Drone.setPosition(1);
        }
        else{
            Drone.setPosition(0.7);
        }



















       /* while(!frontDrive){
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
            if (gamepad1.y) {
                Drone.setPosition(0.3);
            }


            // CLAW ROTATION


            if (LimitSwitch.isPressed()) {
                telemetry.addData("Digital Touch", "Pressed");
            } else {
                telemetry.addData("Digital Touch", " Not Pressed");
            }


            //Slide Goes Up
            if (UpSlideBumper) {
                //BucketHold.setPosition(0);
                SlideR.setPower(0.5);
                SlideL.setPower(0.5);
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
                //   BucketHold.setPosition(0.5);
                SlideR.setPower(0);
                SlideL.setPower(0);
            }
        }





        */
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
