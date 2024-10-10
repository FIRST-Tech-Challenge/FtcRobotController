package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp Program", group="TeleOp")
public class MecanumTeleOp extends OpMode {

    Robot robot = new Robot();
    double ticks = 1425;
    double LM_Ticks; //Find ticks of linear motion motor
    double target;

    public DcMotorEx right_b;
    public DcMotorEx left_f;
    public DcMotorEx right_f;
    public DcMotorEx left_b;


    public Servo specimen_claw;

    public Servo left_servo;

    private boolean servoToggled = false;
    private boolean aPressedLast = false;



    int travel;
    //Code to run ONCE after the driver hits INIT
    @Override
    public void init() {

        robot.init(hardwareMap);

        left_f = hardwareMap.get(DcMotorEx.class, "left_front");
        right_f = hardwareMap.get(DcMotorEx.class, "right_front");
        left_b = hardwareMap.get(DcMotorEx.class, "left_back");
        right_b = hardwareMap.get(DcMotorEx.class, "right_back");
        specimen_claw = hardwareMap.get(Servo.class, "specimen_claw");

        right_f.setDirection(DcMotorSimple.Direction.REVERSE);
        right_b.setDirection(DcMotorSimple.Direction.REVERSE);



        left_servo = hardwareMap.get(Servo.class, "left_servo");




    }
    //Code to run REPEATEDLY after the driver hits INIT
    @Override
    public void init_loop() {

        telemetry.addData("Linear_motion_right", robot.linear_L.linear_motion_left.getCurrentPosition());
        telemetry.addData("left_servo_position", left_servo.getPosition());
        telemetry.update();


        telemetry.addData("linear_motion_left_ticks", robot.linear_L.linear_motion_left.getCurrentPosition());
        telemetry.update();




    }

    //Code to run ONCE after the driver hits PLAY
    @Override
    public void start() {

    }

    //Code to run REPEATEDLY after the driver hits PLAY
    @Override
    public void loop() {



        double x = -gamepad1.right_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = -gamepad1.left_stick_x;


        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        //Hardware needs to change motors for increased speed.

        right_f.setPower( (y + x + rx) / denominator);
        left_b.setPower( (y - x + rx) / denominator);
        left_f.setPower( (y - x - rx) / denominator);
        right_b.setPower( (y + x - rx) / denominator);



        /*
        double[] driveVelocities =
                robot.driveTrain.drive(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        gamepad1.right_stick_x);

        robot.driveTrain.setDriveVelocities(driveVelocities);
        */

        robot.linear_L.linear_motion_left.setPower(gamepad2.left_stick_y);
        robot.linear_R.linear_motion_right.setPower(gamepad2.left_stick_y);

        robot.linear_C.linear_claw.setPower(gamepad2.right_stick_y);

        //Test encoder for linear_motion_left

        if (gamepad1.a){
            travel = 3000;
            while (robot.linear_L.linear_motion_left.getCurrentPosition() < travel){
                robot.linear_L.linear_motion_left.setTargetPosition(travel);
                robot.linear_L.linear_motion_left.setPower(0.8);
                robot.linear_L.linear_motion_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        if (gamepad1.b){
            travel = 0;
            while (robot.linear_L.linear_motion_left.getCurrentPosition() > travel){
                robot.linear_L.linear_motion_left.setTargetPosition(travel);
                robot.linear_L.linear_motion_left.setPower(-0.8);
                robot.linear_L.linear_motion_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }

        robot.linear_C.linear_claw.setPower(gamepad2.right_stick_y);

            if (gamepad1.dpad_right){
                encoder(20,1);
            } else if (gamepad1.dpad_left){
                encoder(-20, 1);

                //-------------------------------------------------


        } /*else if (gamepad1.a){
                robot.left_claw.servo_left.setPosition(1);
        } else if (gamepad1.b){
                robot.left_claw.servo_left.setPosition(-1);
        } */
        else {
            robot.linear_C.linear_claw.setPower(0);

        }

        if (gamepad1.a && !aPressedLast) {
            servoToggled = !servoToggled;
            left_servo.setPosition(servoToggled ? 0.93 : 0.2);
        }
        aPressedLast = gamepad1.a;

        if (gamepad1.b){
                left_servo.setPosition(0);
            }


        if (gamepad2.a){

        }

        else {
            //robot.linear_L.linear_motion_left.setPower(0.005);

            //robot.linear_R.linear_motion_right.setPower(0.005);
            //robot.linear_C.linear_claw.setPower(0);



        }

    }

    //Method to move motor to designated position
    public void encoder(int turnage, double power){
        target = ticks/((double)turnage/100);
        robot.linear_C.linear_claw.setTargetPosition((int) target);
        robot.linear_C.linear_claw.setPower(power);
        robot.linear_C.linear_claw.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
}



