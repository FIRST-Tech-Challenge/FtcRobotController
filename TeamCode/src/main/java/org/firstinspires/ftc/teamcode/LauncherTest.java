package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Launcher Test", group = "Test")
public class LauncherTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor launcher = hardwareMap.get(DcMotor.class, "launcher");

        telemetry.addLine("Launcher found. Press PLAY to spin.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            launcher.setPower(0.5);

            telemetry.addData("Launcher power", launcher.getPower());
            telemetry.update();
        }

        launcher.setPower(0);
    }
}
