package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class DetestmentAndWoeARHAN extends Movable {
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        // Example of controlling movements and then stopping after each step.
        powerWheels(150, "left");
        powerWheels(900, "forward");
        //Place specimen
        powerWheels(20,"back");
        powerWheels(400, "right");
        powerWheels(30,"forward");
        powerWheels(20,"left");
        powerWheels(900,"backward");
        powerWheels(900,"forward");
        powerWheels(10,"right");
        powerWheels(900,"backward");
        turnRobot(200,"right");
        powerWheels(20,"right");
        //Pick up specimen
        turnRobot(200,"right");
        powerWheels(240,"left");
        powerWheels(900,"forward");
        //Place specimen
        powerWheels(10,"back");
        powerWheels(400,"right");
        powerWheels(20,"forward");
        powerWheels(20,"right");
        powerWheels(600,"backward");

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
