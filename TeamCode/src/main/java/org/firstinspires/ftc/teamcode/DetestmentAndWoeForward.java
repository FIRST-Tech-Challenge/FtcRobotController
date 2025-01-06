package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class DetestmentAndWoeForward extends Movable {
    static private DcMotor FLW;
    static private DcMotor BLW;
    static private DcMotor FRW;
    static private DcMotor BRW;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        // Example of controlling movements and then stopping after each step.
        powerWheels(600, "forward");

        // Stop motors after completing the parking routine
        disablePower();
        telemetry.addData("Status", "Completed");
        telemetry.update();
    }

    public void updatePhoneConsole() {
        telemetry.addData("FLW Power", FLW.getPower());
        telemetry.addData("BLW Power", BLW.getPower());
        telemetry.addData("FRW Power", FRW.getPower());
        telemetry.addData("BRW Power", BRW.getPower());
        telemetry.addData("Status", "Running");
        telemetry.update();
    }
}
