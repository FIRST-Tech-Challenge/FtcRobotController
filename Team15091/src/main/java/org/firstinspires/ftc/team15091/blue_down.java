package org.firstinspires.ftc.team15091;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "blue_down", preselectTeleOp = "Gamepad")
public class blue_down extends AutonomousBase{
    @Override
    public void runOpMode() throws InterruptedException {
        setupAndWait();

        robotDriver.gyroDrive(0.6d, 10d, 0d, 2d, null);
        sleep(1000);
        robotDriver.gyroSlide(0.6d, -90d, 90d, 2d, null);
        waitForStart();
    }
}
