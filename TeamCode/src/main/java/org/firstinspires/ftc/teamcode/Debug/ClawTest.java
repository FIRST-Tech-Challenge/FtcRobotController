package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ClawTest extends LinearOpMode {

    /*
    to test claw be spinning
     */

    DcMotor Claw, Slide, BIS;

    public void runOpMode() {

        initRobot();

        // Code that needs to be run once is placed here

        waitForStart();
        while (opModeIsActive()) {
            double graber = gamepad1.right_trigger - gamepad1.left_trigger;
            double slidePow = -gamepad1.left_stick_y;
            if (slidePow > 0 && Slide.getCurrentPosition() > 4250)
            {
                slidePow = 0;
            } else if (slidePow < 0 && Slide.getCurrentPosition() < 0) {
                slidePow = 0;
            }
            jacobsintake(graber);
            slide(slidePow);
            BIS.setPower(-gamepad1.right_stick_y);
        }
    }

    public void initRobot() {
        Claw = hardwareMap.get(DcMotor.class, "Claw");
        Claw.setZeroPowerBehavior(BRAKE);
        Claw.setDirection(FORWARD);
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Slide.setMode(RUN_USING_ENCODER);
        Slide.setZeroPowerBehavior(BRAKE);
        Slide.setDirection(FORWARD);
        BIS = hardwareMap.get(DcMotor.class, "Ben is stupid");
        BIS.setZeroPowerBehavior(BRAKE);
        BIS.setDirection(FORWARD);
    }

    public void jacobsintake(double graber) {
        Claw.setPower(graber);
    }

    public void slide(double power) {
        Slide.setPower(power);
    }
}
