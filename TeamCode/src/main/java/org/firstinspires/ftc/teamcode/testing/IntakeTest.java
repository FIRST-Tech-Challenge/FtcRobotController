package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test", group = "Test")
public class IntakeTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");

        telemetry.addLine("Intake found. Press PLAY to spin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            intake.setPower(0.5);

            telemetry.addData("Intake power", intake.getPower());
            telemetry.update();
        }

        intake.setPower(0);
    }
}
