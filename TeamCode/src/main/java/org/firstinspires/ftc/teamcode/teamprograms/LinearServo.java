package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Linear Servo", group = "Z")
public class LinearServo extends LinearOpMode {

    ServoImplEx linearServo;
    PwmControl.PwmRange range = new PwmControl.PwmRange(900, 2100);

    public void runOpMode() {

        linearServo = hardwareMap.get(ServoImplEx.class, "servoLeft");
        linearServo.setPwmRange(range);

        telemetry.addLine("Press Start");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.x) {
                linearServo.setPosition(0);
            } else if (gamepad1.a) {
                linearServo.setPosition(0.5);
            } else if (gamepad1.b) {
                linearServo.setPosition(1);
            }

            // enable and disable PWM as a kill switch (could have been useful during CENTERSTAGE)
            if (gamepad1.dpad_up) {
                linearServo.setPwmEnable();
            } else if (gamepad1.dpad_down) {
                linearServo.setPwmDisable();
            }

            telemetry.addData("LINEAR SERVO", linearServo.getPosition());
            telemetry.addData("PWM?", linearServo.isPwmEnabled());
            telemetry.update();

        }

    }


}
