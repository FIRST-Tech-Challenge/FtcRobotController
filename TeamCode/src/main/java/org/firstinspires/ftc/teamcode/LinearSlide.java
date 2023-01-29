package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LinearSlide extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        waitForStart();
        //DONT RUN THIS CODE!!!! It's currently malfunctioning, and the values might be incorrect
        if (isStopRequested()) return;

        while (opModeIsActive()) {

            DcMotor liftLeft = hardwareMap.dcMotor.get("liftLeft");
            DcMotor liftRight = hardwareMap.dcMotor.get("liftRight");
            DcMotor fourBarMotor = hardwareMap.dcMotor.get("armMotor");

            double linearPower = gamepad1.left_stick_y;
            double linearPosRight = liftRight.getCurrentPosition();
            double linearPosLeft = liftLeft.getCurrentPosition();

            //Both are placeholder values for now
            double highLinearBarrier = -4500;
            double lowLinearBarrier = 0;

            //100 & -100 are also both placeholder values
            if (linearPosRight >= highLinearBarrier) {
                linearPosRight = highLinearBarrier + 50;
            }
            if (linearPosLeft >= highLinearBarrier) {
                linearPosLeft = highLinearBarrier + 50;
            }
            if (linearPosLeft <= lowLinearBarrier) {
                linearPosLeft = lowLinearBarrier - 50;
            }
            if (linearPosRight <= lowLinearBarrier) {
                linearPosRight = lowLinearBarrier - 50;
            }

            //Figure out which motor is oriented opposite and change that linear pos to a negative (-)
            liftLeft.setPower(-linearPower);
            liftRight.setPower(linearPower);
        }
    }
}