package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Example", preselectTeleOp = "Gamepad")
public class Example extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Wait for the game to start (driver presses PLAY)
        // Abort this loop is started or stopped.
        setupAndWait();

        robotDriver.gyroDrive(0.6d, 24d, 0d, 5d, null);
        robotDriver.gyroTurn(0.6d, 90d, 5d);

        robotDriver.gyroDrive(0.6d, 24d, 90d, 5d, null);
        robotDriver.gyroTurn(0.6d, 180d, 5d);

        robotDriver.gyroDrive(0.6d, 24d, 180d, 5d, null);
    }
}
