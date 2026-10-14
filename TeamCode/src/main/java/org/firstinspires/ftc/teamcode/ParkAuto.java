package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Forward 20cm Right 20cm", group="Robot") // todo: change the name that shows up on the driver station
public class ParkAuto extends LinearOpMode {
    @Override
    public void runOpMode() {
        RockinBot robot = new RockinBot(this);

        telemetry.addData("Autonomous Ready", "Forward and right 20 cm diagonally");
        telemetry.addData("This code was last updated", "10/11/2026, 6:43 pm");
        telemetry.update();
        waitForStart();

        robot.driveToPos(100, 100, 0, 10, 2, 4);

        telemetry.addData("Autonomous", "Complete");
        telemetry.update();
    }
}