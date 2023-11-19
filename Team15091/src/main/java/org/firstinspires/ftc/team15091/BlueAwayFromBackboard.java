package org.firstinspires.ftc.team15091;
// turns: left angles are positive, right angles are negative
// slides: left distances are positive, right distances are negative
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@Autonomous(name = "BlueAwayFromBackboard", preselectTeleOp = "Gamepad")
public class BlueAwayFromBackboard extends AutonomousBase{
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();
        DistanceDetector frontDistance = new DistanceDetector((DistanceSensor)(hardwareMap.get("sensor_front")), 10, false);
        PixelPosition initialPos = rbProcessor.position;
        if (initialPos == PixelPosition.Left) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 24d, 45, 3, null);
            robot.togglePixelHolder(true); // release pixel
            if (should_park) {
                sleep(500);
                robotDriver.gyroDrive(0.3, -10, 45, 3, null); // move backward
                robotDriver.gyroTurn(0.2, 90, 3);
                robotDriver.gyroSlide(0.2, -37.5, 90, 5, null);
                robotDriver.gyroDrive(0.3d, 90, 90, 10, frontDistance); // move forward and park
            }
        }
        else if (initialPos == PixelPosition.Right) {
            robotDriver.gyroDrive(0.3d, 12d, 0, 3, null);
            robotDriver.gyroDrive(0.2d, 25d, -45, 3, null);
            robot.togglePixelHolder(true); // release pixel
            if (should_park) {
                sleep(500);
                robotDriver.gyroDrive(0.3, -10, 45, 3, null); // move backward
                robotDriver.gyroTurn(0.15d, 0, 5); // turn left
                robotDriver.gyroSlide(0.2, 4, 0, 3, null);
                robotDriver.gyroDrive(0.3d, 37.5, 0, 3, null); // drive forward one tile
                robotDriver.gyroTurn(0.15d, 90, 5); // turn left another 90 deg
                robotDriver.gyroDrive(0.3d, 90, 90, 10, frontDistance); // move forward and park
            }
        }
        else { // pixel in the middle position
            robotDriver.gyroDrive(0.3d, 29.5, 0, 3, null); // move forward 1 1/2 tiles, placing the pixel at the spike mark
            robot.togglePixelHolder(true); // release pixel
            sleep(500);
            if (should_park) {
                robotDriver.gyroDrive(0.3d, -6, 0, 3, null); // move back, releasing the pixel
                robotDriver.gyroSlide(0.3d, -27.5, 0, 3, null); // slide one tile to the right
                robotDriver.gyroDrive(0.3d, 22.5, 0, 3, null); // drive one tile forward
                robotDriver.gyroTurn(0.15d, 90, 5); // turn left
                robotDriver.gyroDrive(0.3d, 115, 90, 10, frontDistance); // move forward and park
            }
        }
    }
}
