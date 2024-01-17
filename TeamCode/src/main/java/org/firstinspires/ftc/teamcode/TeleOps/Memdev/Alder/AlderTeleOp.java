package org.firstinspires.ftc.teamcode.TeleOps.Memdev.Alder;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class AlderTeleOp extends LinearOpMode {

    private DcMotor motor;
    private Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("Motor");
        servo = hardwareMap.servo.get("Servo");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y;
            motor.setPower(power);
            servo.setPosition(90);
        }
    }
}
