package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Drive extends LinearOpMode{
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor fLeft  = hardwareMap.dcMotor.get("fLeft");
        DcMotor bLeft = hardwareMap.dcMotor.get("bLeft"); //yaya kaitlyn was here :)
        DcMotor fRight = hardwareMap.dcMotor.get("fRight");
        DcMotor bRight  = hardwareMap.dcMotor.get("bRight");
       //  DcMotor arm = hardwareMap.dcMotor.get("arm");

       // Servo servo1 = hardwareMap.servo.get("servo1");
        // chloe's territory
        // Servo servo2 = hardwareMap.servo.get("servo2");


        fLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        bLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        fRight.setDirection(DcMotorSimple.Direction.REVERSE);
        bRight.setDirection(DcMotorSimple.Direction.FORWARD);
       // arm.setDirection(DcMotorSimple.Direction.FORWARD);

       //  double servoOnePos = 0.5;
       //  double servoTwoPos = 0.5;


        waitForStart();

        while (opModeIsActive()) {
            double forwardMovement = -gamepad1.left_stick_y / 4; //vivian was here hehehe
            double turnMovement = gamepad1.right_stick_x /4;
            double mecanumMovement = gamepad1.left_stick_x /4;
           //  double armPower = gamepad1.right_stick_y;

            double fLeftPower = forwardMovement + turnMovement + mecanumMovement;
            double bLeftPower = forwardMovement + turnMovement - mecanumMovement;
            double fRightPower = forwardMovement - turnMovement - mecanumMovement;
            double bRightPower = forwardMovement - turnMovement + mecanumMovement;

            double maxPower = maxAbsValueDouble(fLeftPower, bLeftPower, fRightPower, bRightPower);

            if (Math.abs(maxPower) > 1) {
                double scale = Math.abs(maxPower);

                fLeftPower /= scale;
                bLeftPower /= scale;
                fRightPower /= scale;
                bRightPower /= scale;



            }

            fLeft.setPower(fLeftPower);
            bLeft.setPower(bLeftPower);
            fRight.setPower(fRightPower);
            bRight.setPower(bRightPower);
            // arm.setPower(armPower);

            /* if (gamepad1.left_bumper == true) {
                servoOnePos += 0.025;
                servoTwoPos -= 0.025;
            }

            if (gamepad1.right_bumper == true){
                servoOnePos -= 0.025;
                servoTwoPos += 0.025;
            }


            servo1.setPosition(servoOnePos);
            servo2.setPosition(servoTwoPos);

             */
        }



    }

    public double maxAbsValueDouble(double a, double... others) {
        double max = a;

        for (double next : others) {
            if (Math.abs(next) > Math.abs(a)) {
                    max = next;
            }
        }
        return max;
    }

}

