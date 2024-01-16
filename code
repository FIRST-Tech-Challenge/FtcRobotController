package org.firstinspires.ftc.teamcode.teleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class centerCodePart2 extends LinearOpMode {
    @Override
    public void runOpMode() {

        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight"); // input 0
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight"); // input 1
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft"); // input 2
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");  // input 3

        DcMotor motorArmExtender = hardwareMap.dcMotor.get("motorArmExtender"); // Ex 0
        DcMotor motorMount = hardwareMap.dcMotor.get("motorMount"); // Ex 1
        DcMotor motorDroneShooter = hardwareMap.dcMotor.get("motorDroneShooter"); // Ex2

        Servo servoArmClaw = hardwareMap.servo.get("servoArmClaw"); // servo 0
        Servo servoArmVertical = hardwareMap.servo.get("servoArmVertical"); // servo 1
        //Servo servoDroneShooter = hardwareMap.servo.get("ServoDroneShooter"); // servo 2
        //Servo servoRobotMount = hardwareMap.servo.get("servoRobotMount"); // servo 3


        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(BRAKE);
        motorFrontRight.setZeroPowerBehavior(BRAKE);
        motorBackLeft.setZeroPowerBehavior(BRAKE);
        motorBackRight.setZeroPowerBehavior(BRAKE);

        motorArmExtender.setZeroPowerBehavior(BRAKE);
        motorMount.setZeroPowerBehavior(BRAKE);
        motorDroneShooter.setZeroPowerBehavior(BRAKE);

        //set motors to zero while not in use(stops robot from moving while joystick isn't pressed)
        waitForStart();


        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);




            double extenderPowerOut = (gamepad2.right_trigger);
            if (gamepad2.right_trigger > 0.01) {
                motorArmExtender.setPower(extenderPowerOut);
            }
            else {
                motorArmExtender.setPower(0);
            }
            double extenderPowerIn = (gamepad2.left_trigger);
            if (gamepad2.left_trigger > 0.01) {
                motorArmExtender.setPower(extenderPowerIn);
            }
            else {
                motorArmExtender.setPower(0);
            }




            double mountPower = gamepad2.left_stick_y;
            if (gamepad2.right_stick_y > 0.1 || gamepad2.right_stick_y < -0.1) {
                motorMount.setPower(mountPower);
            }
            else {
                motorMount.setPower(0);
            }




            if (gamepad2.y) {
                motorDroneShooter.setPower(1); }
            else {
                motorDroneShooter.setPower(0);
            }




            /*
            if (gamepad2.dpad_up) {
                servoArmVertical.setPosition(0);
            } else if (gamepad2.dpad_down) {
                servoArmVertical.setPosition(1);
            }
            */



            if (gamepad2.dpad_up) {
                while (gamepad2.dpad_up) {
                    double position = servoArmVertical.getPosition();
                    servoArmVertical.setPosition(position + 0.01);
                }
            } else {
                servoArmVertical.setPosition(0);
            }
            if (gamepad2.dpad_down) {
                while (gamepad2.dpad_down) {
                    double position = servoArmVertical.getPosition();
                    servoArmVertical.setPosition(position - 0.01);
                }
            } else {
                servoArmVertical.setPosition(0);
            }


            //s
            if (gamepad2.x) {
                servoArmClaw.setPosition(0);
            } else if (gamepad2.b) {
                servoArmClaw.setPosition(1);
            }
            //e




            /*
            if (gamepad2.y) {
                servoDroneShooter.setPosition(1); }
            else {
                servoDroneShooter.setPosition(0);
            }

            if (gamepad2.a) {
                servoRobotMount.setPosition(1); }
            else {
                servoRobotMount.setPosition(0);
            }


            */
        }
    }
}

