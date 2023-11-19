package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "RedAwayFromBackboard", preselectTeleOp = "Gamepad")
public class RedAwayFromBackboard extends AutonomousBase {
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 10, false);
        PixelPosition initialPos = rbProcessor.position;
        if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.3d, 25.5, 0, 3, null); // move forward to the center of the second tile
            robotDriver.gyroTurn(0.1d, -90, 5); // turn right 90 deg
            robotDriver.gyroDrive(0.3d, 3, -90, 3, null); // move forward, placing the pixel on the spike mark
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            if (should_park) {
                robotDriver.gyroDrive(0.3d, -3, -90, 3, null); // move back, resetting the robot position to the center of the second tile
                robotDriver.gyroSlide(0.3d, 25.5, -90, 3, null); // slide left one tile (adjusted by +3 due to hitting the metal bar)
                robotDriver.gyroDrive(0.3d, 90, -90, 10, frontDistance); // move forward and park
            }

        }
        else if (initialPos == PixelPosition.Left) {
            robotDriver.gyroDrive(0.3d, 25.5, 0, 3, null); // move forward to the center of the second tile
            robotDriver.gyroTurn(0.1d, 90, 5); // turn left 90 deg
            robotDriver.gyroDrive(0.3d, 3, 90, 3, null); // move forward, placing the pixel on the spike mark
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            if (should_park) {
                robotDriver.gyroDrive(0.3d, -3, 90, 3, null); // move back, resetting the robot position to the center of the second tile
                robotDriver.gyroTurn(0.15d, 0, 5); // turn right 90 deg
                robotDriver.gyroDrive(0.3d, 22.5, 0, 3, null); // drive forward one tile
                robotDriver.gyroTurn(0.15d, -90, 5); // turn right another 90 deg
                robotDriver.gyroDrive(0.3d, -90, 90, 10, frontDistance); // move forward and park
            }
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.3d, 28, 0, 3, null); // move forward 1 1/2 tiles, placing the pixel at the spike mark
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            if (should_park) {
                robotDriver.gyroDrive(0.3d, -3, 0, 3, null); // move back, releasing the pixel
                robotDriver.gyroSlide(0.3d, 22.5, 0, 3, null); // slide one tile to the left
                robotDriver.gyroDrive(0.3d, 22.5, 0, 3, null); // drive one tile forward
                robotDriver.gyroTurn(0.15d, -90, 5); // turn right
                robotDriver.gyroDrive(0.3d, 115, -90, 10, frontDistance); // move forward and park
            }
        }
    }
}

