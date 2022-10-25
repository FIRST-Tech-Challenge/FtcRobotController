package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "aaa (Blocks to Java)")
public class aaa extends LinearOpMode {

    private RevBlinkinLedDriver blinkin;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        blinkin = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED);
                telemetry.update();
            }
        }
    }
}