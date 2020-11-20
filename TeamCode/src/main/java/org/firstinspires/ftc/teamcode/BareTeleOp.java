package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Bare TeleOP")
public class BareTeleOp extends LinearOpMode {

    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;
    private DcMotor outtakeRight, outtakeLeft;
    private Servo flipper;


    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        flipper = hardwareMap.servo.get("flipper");

        //launcher
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");


        //reverse the needed motors
        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //reverse one of the outtakes
        outtakeLeft.setDirection(DcMotor.Direction.REVERSE);

        double powerMod = 1.0;
        double intakeMod = 1.0;

        waitForStart();

        while (opModeIsActive()) {
            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if (gamepad1.right_bumper) {
                powerMod = 0.5;
            } else {
                powerMod = 1.0;
            }

            //everything driving
            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.sin(angle);
            double powerTwo = r * Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerOne + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerTwo + (rotation))*powerMod);

            //outtake
            double outtakePower = (gamepad2.right_trigger * -0.25);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);

            //flipper
            if (gamepad2.b) {

                    flipper.setPosition(1);

            }

            if(gamepad2.a){
                flipper.setPosition(0);
            }



        }
    }
}
