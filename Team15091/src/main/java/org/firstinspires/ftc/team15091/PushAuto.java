package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Push Pixel Only", group = "Testing", preselectTeleOp = "Gamepad")
public class PushAuto extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();

        PixelPosition initialPos = pixelDetector.objectDetected();
        if (initialPos == PixelPosition.Left) {
            robotDriver.gyroDrive(0.5d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, 45, 3, null);
        } else if (initialPos == PixelPosition.Middle) {
            robotDriver.gyroDrive(0.5d, 30d, 0, 3, null);
        } else { // only option is Right

            robotDriver.gyroDrive(0.5d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 26d, -45, 3, null);
        }

        robotDriver.gyroDrive(0.2d, -26d, 0, 3, null);
    }
}
