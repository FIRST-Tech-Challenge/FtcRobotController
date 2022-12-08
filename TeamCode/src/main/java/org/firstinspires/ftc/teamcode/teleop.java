package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
public class Servotesting extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    private CRServo Right;
    private CRServo Left;
    private DcMotor Crain;
    private DcMotor Spin;


    public void runOpMode() throws InterruptedException {



        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        Left = hardwareMap.get(CRServo.class, "Lefts");
        Crain = hardwareMap.get(DcMotor.class, "Crane");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            double turn;
            double throttle;
            boolean strafeLeft;
            boolean strafeRight;

            float pickup;
            float dropoff;
            boolean spinpowerup;
            boolean spinpowerdown;
            double crainpower;
            boolean turning;

            throttle = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafeLeft = gamepad1.left_bumper;
            strafeRight = gamepad1.right_bumper;

            crainpower = gamepad2.right_stick_y;
            spinpowerup = gamepad2.dpad_right;
            spinpowerdown =gamepad2.dpad_left;
            pickup = gamepad2.left_trigger;
            dropoff = gamepad2.right_trigger;
            turning = gamepad2.b;

            if (strafeLeft) {
                frontLeft.setPower(-0.9);
                frontRight.setPower(-1);
                backLeft.setPower(1);
                backRight.setPower(1);
            }
            if (strafeRight) {
                frontLeft.setPower(0.9);
                frontRight.setPower(1);
                backLeft.setPower(-1);
                backRight.setPower(-1);
            }


            frontLeft.setPower(throttle);
            frontRight.setPower(throttle);
            backLeft.setPower(throttle);
            backRight.setPower(throttle);

            frontLeft.setPower(-turn);
            frontRight.setPower(turn);
            backLeft.setPower(-turn);
            backRight.setPower(turn);

            Crain.setPower(crainpower);

            if (spinpowerup){
                Spin.setPower(.5);
            }
            if (spinpowerdown){
                Spin.setPower(-.5);
            }

            if (!spinpowerdown && !spinpowerup){
                Spin.setPower(0);
            }

            if (pickup>0) {

                Left.setPower(-1);
            }

            if (dropoff>0){

                Left.setPower(1);

            }

            if (dropoff == 0 && pickup == 0){

                Left.setPower(0);

            }


            if (turning){
                Spin.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Crain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Spin.setTargetPosition(0);
                Crain.setTargetPosition(0);
                Spin.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Crain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Spin.setPower(1);
                Crain.setPower(1);
                while(Spin.isBusy()){

                }
                Crain.setTargetPosition(1);
                Crain.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Crain.setPower(1);
                while (Crain.isBusy()){

                }
            }

        }


    }
}