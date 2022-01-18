package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

//@Disabled
@Autonomous(name = "ODTest")
public class PushOdometryTest extends AutonomousTemplate {


    @Override
    public void opModeMain() throws InterruptedException {
        this.initDriveSystem();
        this.initOdometryServos();
        podServos.lower();
        odometry.setPosition(0, 0, 0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("PosX", odometry.returnRelativeXPosition());
            telemetry.addData("PosY", odometry.returnRelativeYPosition());
            telemetry.addData("Rot", odometry.returnOrientation());
            telemetry.update();
        }
    }
}
