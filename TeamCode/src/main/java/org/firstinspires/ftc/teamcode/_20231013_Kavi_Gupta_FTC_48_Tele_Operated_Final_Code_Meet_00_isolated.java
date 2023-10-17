package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele Operated Meet 0 Final iso")
@com.qualcomm.robotcore.eventloop.opmode.Disabled
public class _20231013_Kavi_Gupta_FTC_48_Tele_Operated_Final_Code_Meet_00_isolated extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        DcMotor lift = hardwareMap.dcMotor.get("lift");
        Servo droneServo = hardwareMap.servo.get("drone");
        Servo wristServo = hardwareMap.servo.get("wrist");
        Servo clawServo = hardwareMap.servo.get("claw");


        lift.setDirection(DcMotorSimple.Direction.FORWARD);
        droneServo.setDirection(Servo.Direction.FORWARD);
        wristServo.setDirection(Servo.Direction.FORWARD);
        clawServo.setDirection(Servo.Direction.FORWARD);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        droneServo.scaleRange(0, 1);
        droneServo.setPosition(0.85);
        wristServo.scaleRange(0, 1);
        int liftTargetPosition = 5;
        double wristTargetPos = 0;
        double claw = 0;

        waitForStart();

        while (opModeIsActive()) {
            //gamepad 1 - drive base


            // Rotate the movement direction counter to the bot's rotation



            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]




            //gamepad 2 - mechanisms
            //lift code
            double liftPower = lift.getPower();

            if (gamepad2.right_trigger > 0) {
                liftTargetPosition += 5;
            }else if (gamepad2.left_trigger > 0) {
                liftTargetPosition -= 5;
            }
            if (liftTargetPosition > 962) {
                liftTargetPosition = 957;
            }
            if (liftTargetPosition < 0) {
                liftTargetPosition = 5;
            }

            lift.setTargetPosition(liftTargetPosition);
            lift.setPower(1);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //wrist code

            if (gamepad2.left_bumper){
                wristTargetPos = 1;
            } else if (gamepad2.right_bumper){
                wristTargetPos = 0;
            }
            wristServo.setPosition(wristTargetPos);

            /*
            if (lift.getCurrentPosition() >= 0 && lift.getCurrentPosition() < 500) {
                wristTargetPos = 1;
                wristServo.setPosition(wristTargetPos);

            } else if (lift.getCurrentPosition() >= 500) {
                wristTargetPos = 0;
                wristServo.setPosition(wristTargetPos);
            }

             */

            if (gamepad2.dpad_up) {
                claw = 0;
                clawServo.setPosition(claw);
            }else if (gamepad2.dpad_down) {
                claw = 0.5;
                clawServo.setPosition(claw);
            }

            //drone launcher code

            double droneServoPosition = droneServo.getPosition();

            if (gamepad2.a) {
                droneServo.setPosition(1);
                sleep(1500);
                droneServo.setPosition(0.85);
            }

            // ADDED CODE - sends info about current servo position to driver station
            /*
            telemetry.addData("Drone Servo Position: ", droneServoPosition);

            telemetry.addData("Wrist Position: ", wristServo.getPosition());
            telemetry.addData("Wrist Target: ", wristTargetPos);

            telemetry.addData("Claw Position: ", clawServo.getPosition());
            telemetry.addData("Claw Target: ", claw);

            telemetry.addData("Lift position: ", lift.getCurrentPosition());
            telemetry.addData("Lift power: ", liftPower);
            telemetry.addData("Lift Target Position Requested: ", liftTargetPosition);
            telemetry.addData("Lift Actual Target Position: ", lift.getTargetPosition());


            telemetry.update();

             */
        }
    }
}