package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TestAutonomous", group="A")
public class TestAutonomous extends DriveMethods {

    @Override
    public void runOpMode() {
initMotorsBlue();
calibrateNavXIMU();

        waitForStart();


        while (opModeIsActive()) {

            telemetry.addLine("Cumulative Z: " + getCumulativeZ());
            telemetry.addLine("Regular Z: " + getCurrentZ());
            telemetry.update();
        }
    }
}
