package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Ring Prototype Test
 * 
 * 3 October 2020
 */

@TeleOp(name = "Ring Prototype Test")
public class RingPrototypeTest extends LinearOpMode{
    private DcMotor motorFrontRight, motorFrontLeft, motorBackLeft, motorBackRight;

    private DcMotor intake, outtake;

    //Figures for telemetry calculations
    private static final int OUTTAKE_MOTOR_RPM = 1100;
    private static final double OUTTAKE_GEAR_RATIO = 3.0;
    private static final double OUTTAKE_WHEEL_RADIUS_IN = 2;
    private static final double OUTTAKE_WHEEL_RADIUS_M = OUTTAKE_WHEEL_RADIUS_IN*0.0254;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");

        intake = hardwareMap.dcMotor.get("intake");
        outtake = hardwareMap.dcMotor.get("outtake");

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerMod = 1.0;

        double intakeMod = 1.0;
        double outtakeMod = 1.0;

        waitForStart();

        while(opModeIsActive()){
            /*
            Checks if right bumper is pressed. If so, power is reduced
             */
            if(gamepad1.right_bumper){
                powerMod = 0.5;
            }else{
                powerMod = 1.0;
            }

            /*
            Change direction of intake
            */
            if(gamepad2.a){
                intakeMod = -1.0;
            }else{
                intakeMod = 1.0;
            }

            /*
            Ability to test a variety of outtake motor speeds
            */
            if(gamepad2.dpad_up){
                if(outtakeMod != 1.0){
                    outtakeMod += 0.1;
                }
            }

            if(gamepad2.dpad_down){
                if(outtakeMod != 0.0){
                    outtakeMod -= 0.1;
                }
            }

            //Mecanum drive using trig
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI/4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r*Math.sin(angle);
            double powerTwo = r*Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation))*powerMod);
            motorFrontRight.setPower((powerTwo + (rotation))*powerMod);
            motorBackLeft.setPower((powerTwo - (rotation))*powerMod);
            motorBackRight.setPower((powerOne + (rotation))*powerMod);

            intake.setPower(gamepad1.left_trigger * intakeMod);


            //Sending data on power of outtake, outtake motor RPM, and tangential velocity of outtake wheel to telemetry
            double outtakePower = (gamepad1.right_trigger * outtakeMod);
            outtake.setPower(outtakePower);

            double outtakeRPM = outtakePower * OUTTAKE_MOTOR_RPM * OUTTAKE_GEAR_RATIO;
            double outtakeWheelVelocity = (outtakeRPM * 2 * Math.PI * OUTTAKE_WHEEL_RADIUS_M)/60;

            telemetry.addData("Outtake Power", outtakePower);
            telemetry.addData("Outtake RPM", outtakeRPM);
            telemetry.addData("Outtake Wheel Velocity (m/s)", outtakeWheelVelocity);

            telemetry.update();
            idle();
        }
    }    
}
