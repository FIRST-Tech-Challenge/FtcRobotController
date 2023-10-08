package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorTest extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    @Override

    public void runOpMode() throws InterruptedException{
        frontLeft = hardwareMap.get(DcMotor.class, "frontleft");
        frontRight = hardwareMap.get(DcMotor.class, "frontright");
        backLeft = hardwareMap.get(DcMotor.class, "backleft");
        backRight = hardwareMap.get(DcMotor.class, "backright");
        waitForStart();
        if (isStopRequested())
            return;
        while (opModeIsActive()) {
            if (gamepad1.b) {
                if (gamepad1.right_trigger>0) {
                    frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
                    backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    backRight.setDirection(DcMotorSimple.Direction.FORWARD);
                    frontLeft.setPower(gamepad1.right_trigger);
                    frontRight.setPower(gamepad1.right_trigger);
                    backLeft.setPower(gamepad1.right_trigger);
                    backRight.setPower(gamepad1.right_trigger);
                }
                else {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);
                }
                if (gamepad1.left_trigger>0) {
                    frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                    backRight.setDirection(DcMotorSimple.Direction.REVERSE);
                    frontLeft.setPower(gamepad1.left_trigger);
                    frontRight.setPower(gamepad1.left_trigger);
                    backLeft.setPower(gamepad1.left_trigger);
                    backRight.setPower(gamepad1.left_trigger);
                }
                else {
                    frontLeft.setPower(0);
                    frontRight.setPower(0);
                    backLeft.setPower(0);
                    backRight.setPower(0);

                }
            }


            }
        }
    }
