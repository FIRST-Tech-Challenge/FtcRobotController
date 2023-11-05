package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Push Pixel Only", group = "Testing", preselectTeleOp = "Gamepad")
public class PushAuto extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();

        PixelPosition initialPos = pixelDetector.objectDetected();
        if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.5d, 9d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, -45, 3, null);
        } else if (initialPos == PixelPosition.Middle) {
            robotDriver.gyroDrive(0.5d, 30d, 0, 3, null);
        } else {
            robotDriver.gyroDrive(0.5d, 9d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, 45, 3, null);
        }

        robotDriver.gyroDrive(0.2d, -25d, 0, 3, null);
    }
}
