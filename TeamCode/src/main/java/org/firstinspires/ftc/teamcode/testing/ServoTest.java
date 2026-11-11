package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Test")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo leftServo = hardwareMap.get(Servo.class, "frontLeft");
        Servo rightServo = hardwareMap.get(Servo.class, "frontRight");

        telemetry.addLine("Servos found. Press PLAY.");
        telemetry.update();

        waitForStart();

        double position = 0.0;
        double step = 0.02;
        while (opModeIsActive()) {
            position += step;
            if (position >= 1.0) {
                position = 1.0;
                step = -step;
            } else if (position <= 0.0) {
                position = 0.0;
                step = -step;
            }

            leftServo.setPosition(position);
            rightServo.setPosition(position);

            telemetry.addData("Commanded position", "%.2f", position);
            telemetry.update();

            sleep(20);
        }
    }
}
