package org.firstinspires.ftc.teamcode.kuykendall;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name="IntakeTest", group="Test")
public class intake1 extends OpMode {

    private Servo wristServo;
    private CRServo intServo;
    private boolean intServoState = false;
    private boolean intServoReverse = false;

    @Override
    public void init() {
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        intServo = hardwareMap.get(CRServo.class, "intServo");
        wristServo.setPosition(0.3); // Initial position for pickup
    }

    @Override
    public void loop() {
        // Toggle continuous servo on 'x' press
        if (gamepad1.x) {
            intServoState = !intServoState;
            if (intServoState) {
                intServo.setPower(2.0); // Spin forward
            } else {
                intServo.setPower(0); // Stop spinning
            }
        }

        // Reverse continuous servo on 'b' press
        if (gamepad1.b) {
            intServoReverse = !intServoReverse;
            if (intServoReverse) {
                intServo.setPower(-2.0); // Spin backward
            } else {
                intServo.setPower(0); // Stop spinning
            }
        }

        // Set wristServo to 'pickup' position on 'a' press
        if (gamepad1.a) {
            wristServo.setPosition(0.4);
        }

        // Set wristServo to 'dropoff' position on 'y' press
        if (gamepad1.y) {
            wristServo.setPosition(.8);
        }
    }
}

