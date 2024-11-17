package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="BlueLeft", group="Auto")
public class BlueLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        if (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.update();


        }
    }

}
// 4300