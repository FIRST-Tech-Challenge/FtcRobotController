package org.firstinspires.ftc.masters.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.CSCons;

@TeleOp(name = "Vslidetest")
public class verticalSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor backSlides = hardwareMap.dcMotor.get("backSlides");

        Servo outtakeRotation = hardwareMap.servo.get("outtakeRotation");
        Servo outtakeMovementRight = hardwareMap.servo.get("outtakeMovementRight");
        Servo outtakeMovementLeft = hardwareMap.servo.get("outtakeMovementLeft");
        Servo outtakeHook = hardwareMap.servo.get("outtakeHook");

        backSlides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        outtakeMovementLeft.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeMovementRight.setPosition(CSCons.outtakeMovementBackTransfer);
        outtakeRotation.setPosition(CSCons.outtakeAngleTransfer);
        outtakeHook.setPosition(CSCons.openHook);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                backSlides.setPower(1);
            } else if (gamepad1.dpad_down) {
                backSlides.setPower(-.8);
            } else {
                backSlides.setPower(0);
            }


        }
    }
}
