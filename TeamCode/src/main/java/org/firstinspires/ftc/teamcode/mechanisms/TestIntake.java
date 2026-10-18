package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Test Intake", group = "Test")
public class TestIntake extends LinearOpMode {
    DcMotor intake;
    CRServo intakeServo1;
    CRServo intakeServo2;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = hardwareMap.get(DcMotor.class, "Intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeServo1 = hardwareMap.get(CRServo.class, "servo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "servo2");

        waitForStart();
        if(isStopRequested()) return;

        double power;
        power = 0;
        boolean lastLeftBumper = false;
        boolean lastRightBumper = false;

        try {
        while(opModeIsActive()){
            if(gamepad1.left_bumper && !lastLeftBumper) {
                power += 0.1;
} else if(gamepad1.right_bumper && !lastRightBumper) {
                power -= 0.1;
}
            lastLeftBumper = gamepad1.left_bumper;
            lastRightBumper = gamepad1.right_bumper;
            power = Range.clip(power, 0.0, 1.0);

            if(gamepad1.a) {
                intake.setPower(power);
    } else if(gamepad1.b) {
                intake.setPower(-power);
    } else {
                intake.setPower(0);
           }if (gamepad1.x) {
                intakeServo1.setPower(1.0);
                intakeServo2.setPower(-1.0);
           }else if (gamepad1.y) {
                intakeServo1.setPower(-1.0);
                intakeServo2.setPower(1.0);
           }else {
                intakeServo1.setPower(0);
                intakeServo2.setPower(0);
            }

            telemetry.addData("Power", power);
            telemetry.update();
            }
        } finally {
            intake.setPower(0);
            intakeServo1.setPower(0);
            intakeServo2.setPower(0);
        }
}
}
