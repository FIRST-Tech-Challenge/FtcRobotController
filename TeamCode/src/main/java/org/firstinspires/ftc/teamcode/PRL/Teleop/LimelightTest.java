package org.firstinspires.ftc.teamcode.PRL.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.PRL.Class.LimelightClass;

@TeleOp()
public class LimelightTest extends LinearOpMode {

    private LimelightClass limelight;

    @Override
    public void runOpMode() {
        limelight = new LimelightClass(hardwareMap);
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelight.update();
            limelight.telemetry(telemetry);
            telemetry.update();
        }

        limelight.stop();
    }
}
