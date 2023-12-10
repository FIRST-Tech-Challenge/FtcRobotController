package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "testForArm")
public class testThingyIG extends LinearOpMode {

    DcMotorEx armMotorThing;
    @Override
    public void runOpMode() throws InterruptedException {
        armMotorThing = hardwareMap.get(DcMotorEx.class, "armMotor");
        waitForStart();
        while (opModeIsActive()) {
            armMotorThing.setPower(gamepad1.left_stick_y);
            this.telemetry.addLine("thing: " + gamepad1.left_stick_y);
            this.telemetry.update();
        }
    }
}
