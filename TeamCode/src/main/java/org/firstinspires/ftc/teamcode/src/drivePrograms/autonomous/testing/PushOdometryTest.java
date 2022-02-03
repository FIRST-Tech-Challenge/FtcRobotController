package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

@Disabled
@Autonomous(name = "ODTest")
public class PushOdometryTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        TeleopDriveTrain dt = new TeleopDriveTrain(hardwareMap, frontRightName, frontLeftName, backRightName, backLeftName);
        dt.setDrivePowerMult(.3);
        this.initDriveSystemWithWheelDrift();

        this.initOdometryServos();
        podServos.lower();

        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            dt.setPowerFromGamepad(gamepad1);

            telemetry.addData("PosX", gps.getX());
            telemetry.addData("PosY", gps.getY());
            telemetry.addData("Rot", gps.getRot());
            telemetry.update();
        }
    }
}
