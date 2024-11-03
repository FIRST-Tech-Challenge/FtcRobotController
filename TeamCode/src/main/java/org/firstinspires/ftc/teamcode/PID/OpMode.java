package org.firstinspires.ftc.teamcode.PID;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class OpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DriveTrain driveTrain = new DriveTrain(hardwareMap);
        waitForStart();

        while (opModeIsActive()) {
            driveTrain.move(0, 12, 0, telemetry);
        }

    }
}
