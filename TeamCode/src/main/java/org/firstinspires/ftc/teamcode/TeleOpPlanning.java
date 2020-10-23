package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name = "AAMainTeleOp")
public class TeleOpPlanning extends LinearOpMode {
    private DcMotor motorFrontRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;
    private DcMotor motorBackRight;

    private CRServo conveyor;
    private DcMotor outtakeRight;
    private DcMotor outtakeLeft;
    private DcMotor intake;

    private static final int OUTTAKE_MOTOR_RPM = 1100;
    private static final double OUTTAKE_GEAR_RATIO = 3.0;
    private static final double OUTTAKE_WHEEL_RADIUS_IN = 2;
    private static final double OUTTAKE_WHEEL_RADIUS_M = OUTTAKE_WHEEL_RADIUS_IN * 0.0254;

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontRight = hardwareMap.dcMotor.get("FR");
        motorFrontLeft = hardwareMap.dcMotor.get("FL");
        motorBackLeft = hardwareMap.dcMotor.get("BL");
        motorBackRight = hardwareMap.dcMotor.get("BR");
        //intake
        //conveyor

        //launcher
        intake = hardwareMap.dcMotor.get("intake");
        outtakeRight = hardwareMap.dcMotor.get("outtakeRight");
        outtakeLeft = hardwareMap.dcMotor.get("outtakeLeft");


        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);

        outtakeRight.setDirection(DcMotor.Direction.REVERSE);


        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double powerMod = 1.0;
        double outtakeMod = 1.0;
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
            //click a button to move in position to launch
            //trigger to launch
            //trigger to intake? and conveyor simultaneously?
            //button to stop top conveyor, middle and top, then all 3 move. Like a mod thing. Lets use x for now
            //when press x, move the conveyor 5 in
//ab up down elevator
            //x
            //
            //outtake
            double outtakePower = (gamepad1.right_trigger * outtakeMod);
            outtakeLeft.setPower(outtakePower);
            outtakeRight.setPower(outtakePower);
            double outtakeRPM = outtakePower * OUTTAKE_MOTOR_RPM * OUTTAKE_GEAR_RATIO;
            double outtakeWheelVelocity = (outtakeRPM * 2 * Math.PI * OUTTAKE_WHEEL_RADIUS_M) / 60;

            //intake
            double intakeSpeed = gamepad1.left_trigger * intakeMod;
            intake.setPower(intakeSpeed);

            //conveyor
            if (gamepad1.x) {
                intake.setPower(1);
                //sleep?
                intake.setPower(0);
            }

            //driving
            double angle = Math.atan2(gamepad1.right_stick_y, gamepad1.right_stick_x) - (Math.PI / 4);
            double r = Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
            double rotation = gamepad1.left_stick_x;

            double powerOne = r * Math.sin(angle);
            double powerTwo = r * Math.cos(angle);

            motorFrontLeft.setPower((powerOne - (rotation)) * powerMod);
            motorFrontRight.setPower((powerTwo + (rotation)) * powerMod);
            motorBackLeft.setPower((powerTwo - (rotation)) * powerMod);
            motorBackRight.setPower((powerOne + (rotation)) * powerMod);

            telemetry.addData("FL Power", motorFrontLeft.getPower());
            telemetry.addData("BL Power", motorBackLeft.getPower());
            telemetry.addData("FR Power", motorFrontRight.getPower());
            telemetry.addData("BR Power", motorBackRight.getPower());

            telemetry.update();
            idle();
        }
    }
}