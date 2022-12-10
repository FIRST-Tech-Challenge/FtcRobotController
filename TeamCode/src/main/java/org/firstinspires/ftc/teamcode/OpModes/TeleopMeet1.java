package org.firstinspires.ftc.teamcode.OpModes;
//imports
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Helper.RobotMeet1;

@TeleOp(name = "22-23 TeleOp", group = "LinearOpMode")

public class TeleopMeet1 extends LinearOpMode {

    //tells you how long the robot has run for
    private ElapsedTime runtime = new ElapsedTime();
    double timeout_ms = 0;

    RobotMeet1 robot = new RobotMeet1();

    //How fast your robot will accelerate.
    public double acceleration = 0.5;

    // holding power for vertical slider.
    private double holdingPower = -0.01;

    //Motor powers
    public double fl_power = 0;
    public double bl_power = 0;
    public double fr_power = 0;
    public double br_power = 0;

    public double DRIVETRAIN_SPEED = 0.6;



    @Override
    public void runOpMode() throws InterruptedException {
        /**
         * Instance of Robot class is initalized
         */
        robot.init(hardwareMap);

        /**
         * This code is run during the init phase, and when opMode is not active
         * i.e. When "INIT" Button is pressed on the Driver Station App
         */

        telemetry.addData("FL Motor Encoder", robot.FLMotor.getCurrentPosition());
        telemetry.addData("BL Motor Encoder", robot.BLMotor.getCurrentPosition());
        telemetry.addData("BR Motor Encoder", robot.BRMotor.getCurrentPosition());
        telemetry.addData("FR Motor Encoder", robot.FRMotor.getCurrentPosition());
        telemetry.addData("VSlider Encoder", robot.vSlider.getCurrentPosition());
        telemetry.addData("Claw Position", robot.claw.getPosition());
        telemetry.update();

        waitForStart();



        while (opModeIsActive()) {

            double move_y_axis = gamepad1.left_stick_y;
            double move_x_axis = -gamepad1.left_stick_x;
            double pivot_turn = -gamepad1.right_stick_x;

            //Sets the target power
            double target_fl_power = move_y_axis + move_x_axis + pivot_turn;
            double target_bl_power = move_y_axis - move_x_axis + pivot_turn;
            double target_fr_power = move_y_axis - move_x_axis - pivot_turn;
            double target_br_power = move_y_axis + move_x_axis - pivot_turn;

            //Adds how far you are from target power, times acceleration to the current power.
            fl_power += acceleration * (target_fl_power - fl_power);
            bl_power += acceleration * (target_bl_power - bl_power);
            fr_power += acceleration * (target_fr_power - fr_power);
            br_power += acceleration * (target_br_power - br_power);

            /**
             Drivetrain
             */

            robot.FLMotor.setPower(DRIVETRAIN_SPEED * fl_power * 0.50);
            robot.BLMotor.setPower(DRIVETRAIN_SPEED * bl_power * 0.50);
            robot.FRMotor.setPower(DRIVETRAIN_SPEED * fr_power * 0.36);
            robot.BRMotor.setPower(DRIVETRAIN_SPEED * br_power * 0.50);

            /** Claw **/
            if(gamepad2.x) {
                robot.claw.setPosition(1);
                sleep(500);
                robot.claw.setPosition(0);
            }
            if(gamepad2.y) {
                robot.claw.setPosition(1);
                telemetry.addData("Claw Position", robot.claw.getPosition());
                telemetry.update();
            }

            /** Slider **/
            double vSliderPower =  gamepad2.left_stick_y * 2;
            double hSliderPower = -gamepad2.right_stick_x * 0.75;
            double vsliderPos = robot.vSlider.getCurrentPosition();

//            if((vSliderPower > 0 && vsliderPos < 15000) || (vSliderPower < 0 && vsliderPos > -100 ))
//            {
//                //Moving up.                                Moving down.
                robot.DriveSlider(-vSliderPower);
//            }
//            else {
//                robot.DriveSlider(0);
//            }

            robot.hSlider.setPower(hSliderPower);


            telemetry.addData("FL Motor Encoder", robot.FLMotor.getCurrentPosition());
            telemetry.addData("BL Motor Encoder", robot.BLMotor.getCurrentPosition());
            telemetry.addData("BR Motor Encoder", robot.BRMotor.getCurrentPosition());
            telemetry.addData("FR Motor Encoder", robot.FRMotor.getCurrentPosition());
            telemetry.addData("vSliderPower", vSliderPower);
            telemetry.addData("vSlider Encoder", vsliderPos);
            telemetry.addData("vSlider2 Encoder", robot.vSlider2.getCurrentPosition());
            telemetry.addData("Claw Position", robot.claw.getPosition());

            telemetry.update();

        }

    }
}
