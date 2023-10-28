package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//add any test stuff you need to do here
@TeleOp
public class TestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo holderClamp = hardwareMap.servo.get("holderClamp");
        Servo arm = hardwareMap.servo.get("arm");
        DcMotor lsBack = hardwareMap.dcMotor.get("lsBack");
        DcMotor lsFront = hardwareMap.dcMotor.get("lsFront");
        DcMotor lFront = hardwareMap.dcMotor.get("fLeft");
        DcMotor rFront = hardwareMap.dcMotor.get("fRight");
        DcMotor lBack = hardwareMap.dcMotor.get("bLeft");
        DcMotor rBack = hardwareMap.dcMotor.get("bRight");

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rBack.setDirection(DcMotorSimple.Direction.FORWARD);
        lBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rFront.setDirection(DcMotorSimple.Direction.FORWARD);
        lFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsFront.setDirection(DcMotorSimple.Direction.REVERSE);
        lsBack.setDirection(DcMotorSimple.Direction.REVERSE);

        lsFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lsBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lsFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lsFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        double holderClampPos = 0.5;
        double armPos = 0.594; //down position

        boolean hangingMode = false;

        while (opModeIsActive()) {

            //GAMEPAD 2 CONTROLS

            double remainingDistanceHigh = 300 - lsFront.getCurrentPosition();
            double remainingDistanceMid = 200 - lsFront.getCurrentPosition();
            double remainingDistanceLow = 100 - lsFront.getCurrentPosition();
            double remainingDistanceZero = -lsFront.getCurrentPosition();

            //set linear slide modes
            if (gamepad2.dpad_up && remainingDistanceHigh > 10) {
                lsFront.setPower(remainingDistanceHigh*0.002);
                lsBack.setPower(remainingDistanceHigh*0.002);
            } else if (gamepad2.dpad_right && remainingDistanceMid > 10) {
                lsFront.setPower(remainingDistanceMid*0.002);
                lsBack.setPower(remainingDistanceMid*0.002);
            } else if (gamepad2.dpad_left && remainingDistanceLow > 10) {
                lsFront.setPower(remainingDistanceLow*0.002);
                lsBack.setPower(remainingDistanceLow*0.002);
            } else if (gamepad2.dpad_down && remainingDistanceZero > 10) {
                lsFront.setPower(remainingDistanceZero*0.002);
                lsBack.setPower(remainingDistanceZero*0.002);
            }

            if(gamepad2.b == true) {
                hangingMode = true;
                //turn on hanging mode to increase power for linear slides
            }

            if(hangingMode == false) {
                //if not hanging, power less
                if (-gamepad2.left_stick_y > 0) {
                    lsBack.setPower(0.5);
                    lsFront.setPower(0.5);
                } else if (-gamepad2.left_stick_y < 0) {
                    lsBack.setPower(-0.5);
                    lsFront.setPower(-0.5);
                } else {
                    lsBack.setPower(0);
                    lsFront.setPower(0);
                }
            } else if (hangingMode == true) {
                //if hanging, power more
                if (-gamepad2.left_stick_y > 0) {
                    lsBack.setPower(1);
                    lsFront.setPower(1);
                } else if (-gamepad2.left_stick_y < 0) {
                    lsBack.setPower(-1);
                    lsFront.setPower(-1);
                } else {
                    lsBack.setPower(0);
                    lsFront.setPower(0);
                }
            }

            telemetry.addLine(String.valueOf(armPos));
            telemetry.addLine(String.valueOf(lsFront.getCurrentPosition()));

            //intake
            if (gamepad2.left_trigger > 0) {
                intake.setPower(1);
                holderClampPos = 0.3;
                //if intake button held, keep holder open
            } else if (gamepad2.left_bumper == true) {
                //reversed intake
                intake.setPower(-1);
                holderClampPos = 0.3;
            } else {
                if (Math.abs(gamepad1.left_stick_x) > 0.1 || Math.abs(gamepad1.left_stick_y) >0.1 || Math.abs(gamepad1.right_stick_x) >0.1) {
                    //if robot moving, keep holder closed
                    holderClampPos = 0.5;
                }
                intake.setPower(0);
            }

            if(gamepad2.right_bumper == true) {
                holderClampPos -= 0.025;
                //open
            } else if (gamepad2.right_trigger > 0) {
                holderClampPos += 0.025;
                //close
            }


            //pivot arm
            if(Math.abs(gamepad2.right_stick_y) > 0.1) {
                armPos -= -(gamepad2.right_stick_y / (1 / 0.004));
            }

            //set limits for the holder clamp
            if(holderClampPos>1) {
                holderClampPos = 1;
            } else if (holderClampPos<0.3) {
                holderClampPos = 0.3;
            }

            //set limits for the arm rotation position
            if(armPos>1) {
                armPos = 1;
            } else if (armPos<0) {
                armPos=0;
            }

            //automatically move arm to intake and outtake position
            if(gamepad2.a == true) {
                armPos = 0.594; //down (inttake) position
            }
            if(gamepad2.y == true) {
                armPos = 0.845; //up (outtake) position
            }

            //GAMEPAD 1 CONTROLS

            double forwardBackward = gamepad1.left_stick_y * -0.5;
            double turning = gamepad1.right_stick_x * 0.5;
            double mecanuming = gamepad1.left_stick_x * 0.5;

            double fLeftPower = forwardBackward + turning + mecanuming;
            double fRightPower = forwardBackward - turning - mecanuming;
            double bLeftPower = forwardBackward + turning - mecanuming;
            double bRightPower = forwardBackward - turning + mecanuming;

            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);

                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;
            }

            lFront.setPower(fLeftPower);
            rFront.setPower(fRightPower);
            lBack.setPower(bLeftPower);
            rBack.setPower(bRightPower);

            holderClamp.setPosition(holderClampPos);
            arm.setPosition(armPos);

            telemetry.addLine(String.valueOf(holderClampPos));
            telemetry.update();
        }
    }

    private double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(max)) {
                max = next;
            }
        }

        return max;
    }
}
