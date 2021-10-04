package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Servo Test", group = "TeleOp")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        CRServo servoIntakeLeft = new CRServo(hardwareMap, "servoIntakeLeft");
        CRServo servoIntakeRight = new CRServo(hardwareMap, "servoIntakeRight");

        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (gamepad1.x) {
                    servoIntakeLeft.set(1.0);
                    servoIntakeRight.set(-1.0);
                } else if (gamepad1.b) {
                    servoIntakeLeft.set(-1.0);
                    servoIntakeRight.set(1.0);
                } else {
                    servoIntakeLeft.set(0.0);
                    servoIntakeRight.set(0.0);
                }
            }
        }
    }
}
