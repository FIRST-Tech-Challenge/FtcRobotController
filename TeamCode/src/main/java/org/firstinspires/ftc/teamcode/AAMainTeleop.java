package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ftdi.eeprom.FT_EEPROM_232H;

@TeleOp(name = "AA Main Teleop")
public class AAMainTeleop extends LinearOpMode {
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight, motorIntake, motorOuttake;
    private CRServo servoDuck;
    private Servo bucket;

    @Override
    public void runOpMode() throws InterruptedException, TargetPositionNotSetException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        motorIntake = hardwareMap.dcMotor.get("intake");
        motorOuttake = hardwareMap.dcMotor.get("outtake");
        servoDuck = hardwareMap.crservo.get("duck");
        bucket = hardwareMap.servo.get("bucket");

        //reverse the needed motors
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //motorOuttake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //motorOuttake.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double powerMod;

        waitForStart();

        while(opModeIsActive()){
            powerMod=1.0;
            /*if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }
            //gamepad 2 does duck, bucket, and outtake. trigger for duck, bucket dpad, outtake joystick

            motorIntake.setPower(gamepad1.left_trigger);
            motorIntake.setPower(-gamepad1.right_trigger);

            servoDuck.setPower(gamepad2.left_trigger);
            servoDuck.setPower(-gamepad2.right_trigger);*/

            if (gamepad1.left_bumper) {
                motorIntake.setPower(powerMod);
                telemetry.addData("yes?", "yes");
            }
            else if (gamepad1.right_bumper) {
                motorIntake.setPower(-powerMod);
            }
            else {
                motorIntake.setPower(0);
            }

            if (gamepad2.left_bumper) {
                servoDuck.setPower(1);
            }
            else if (gamepad2.right_bumper) {
                servoDuck.setPower(-1);
            }
            else {
                motorIntake.setPower(0);
            }

            if(gamepad2.dpad_up){
                bucket.setPosition(bucket.getPosition()+0.01);
            }
            if(gamepad2.dpad_down){
                bucket.setPosition(bucket.getPosition()-0.01);
            }

            motorOuttake.setPower(gamepad2.right_stick_y/2);
            //motorOuttake.setPower(gamepad2.right_stick_y);

            //trying to keep the arm in position and not swinging around
            /*if (gamepad2.x) {
                motorOuttake.setTargetPosition(5);
            }
            else if (gamepad2.a) {
                motorOuttake.setTargetPosition(10);
            }
            else if (gamepad2.b) {
                motorOuttake.setTargetPosition(15);
            }
            else {
                motorOuttake.setTargetPosition(0);
            }*/

            motorOuttake.setPower(gamepad2.right_stick_y);

            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) + (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.cos(angle);
            double powerTwo = r*Math.sin(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo - (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo + (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);

            telemetry.update();
        }
    }
}
