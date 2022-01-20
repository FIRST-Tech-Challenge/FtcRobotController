package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

//@Disabled
@Autonomous(name = "ODTest")
public class PushOdometryTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        TeleopDriveTrain dt = new TeleopDriveTrain(hardwareMap, frontRightName, frontLeftName, backRightName, backLeftName);
        dt.setDrivePowerMult(.3);
        this.initDriveSystemWithWheelDrift();

        this.initOdometryServos();
        podServos.lower();
        //odometry.setPosition(133.5, 103, 180);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            dt.setPowerFromGamepad(gamepad1);

            telemetry.addData("PosX", odometry.returnRelativeXPosition());
            telemetry.addData("PosY", odometry.returnRelativeYPosition());
            telemetry.addData("Rot", odometry.returnOrientation());
            telemetry.update();
        }
    }
}
