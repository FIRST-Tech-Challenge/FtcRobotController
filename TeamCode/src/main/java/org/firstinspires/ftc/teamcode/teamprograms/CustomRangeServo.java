package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Custom Range Servo", group = "Z")
public class CustomRangeServo extends LinearOpMode {

    ServoImplEx customServo;
    PwmControl.PwmRange range = new PwmControl.PwmRange(900, 2100);


    public void runOpMode() {

        customServo = hardwareMap.get(ServoImplEx.class, "servoLeft");
        customServo.setPwmRange(range);

        telemetry.addLine("Press Start");
        telemetry.addLine("SET UP FOR HITEC HSRM9382TH SERVO");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            if (gamepad1.x) {
                customServo.setPosition(0);
            } else if (gamepad1.a) {
                customServo.setPosition(0.5);
            } else if (gamepad1.b) {
                customServo.setPosition(1);
            }

            telemetry.addData("SERVO POS", customServo.getPosition());
            telemetry.addData("PWM?", customServo.isPwmEnabled());
            telemetry.update();



        }

    }


}
