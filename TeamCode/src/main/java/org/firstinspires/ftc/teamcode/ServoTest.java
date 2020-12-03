package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Servo Test", group="Linear Opmode")

public class ServoTest extends LinearOpMode {

    private TwoPosServo claw; //the file this used to be is still called Foundation btw
    private TwoPosServo gear;
    private boolean clawButtonIsDown = false; // controls the claw servo button press
    private boolean gearboxButtonIsDown = false; // controls the gearbox servo button press

    @Override
    public void runOpMode() {

        claw = new TwoPosServo(
                hardwareMap.get(Servo.class, "claw"),
                0.5, 1.0);
        gear = new TwoPosServo(
                hardwareMap.get(Servo.class, "gearbox"),
                0.5, 1.0);


        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.y && !clawButtonIsDown) {
                clawButtonIsDown = true;
                claw.nextPos();
            } else if (!gamepad1.y) {
                clawButtonIsDown = false;
            }

            if (gamepad1.y && !gearboxButtonIsDown) {
                clawButtonIsDown = true;
                gear.nextPos();
            } else if (!gamepad1.y) {
                clawButtonIsDown = false;
            }

            telemetry.addData("Motor Power", gamepad1.left_stick_y);
            telemetry.addData("Right Stick Pos", gamepad1.right_stick_y);
//            telemetry.addData("Servo Position", currentPos);
//            telemetry.addData("Servo Max", max);
//            telemetry.addData("Servo Min", min);
            telemetry.update();

        }
    }
}
