package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Marker Test Only", group = "Testing", preselectTeleOp = "Gamepad")
public class MarkerTest extends AutonomousBase{
    @Override
    public void runOpMode() throws InterruptedException {
        RBDetector detector = new RBDetector(hardwareMap);

        telemetry.addData("Pixel", () -> detector.debug());

        while (!(isStarted() || isStopRequested())) {
            idle();

            telemetry.update();
        }

        detector.dispose();
    }
}
