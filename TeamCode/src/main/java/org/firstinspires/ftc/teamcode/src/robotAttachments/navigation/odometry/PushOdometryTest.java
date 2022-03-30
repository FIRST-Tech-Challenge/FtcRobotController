package org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

//@Disabled
@Autonomous(name = "ODTest")
public class PushOdometryTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        TeleopDriveTrain dt = new TeleopDriveTrain(hardwareMap, frontRightName, frontLeftName, backRightName, backLeftName);
        dt.setDrivePowerMultiplier(.3);
        this.initAllWithWheelDrift();


        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            IMUOdometry imuOdometry = (IMUOdometry) gps;
            dt.gamepadControl(gamepad1, null);

            telemetry.addData("PosX", gps.getX());
            telemetry.addData("PosY", gps.getY());
            telemetry.addData("Rot", gps.getRot());
            telemetry.addData("Right Odometry Pod", imuOdometry.verticalEncoderRight.getCurrentPosition());
            telemetry.addData("Left Odometry Pod", imuOdometry.verticalEncoderLeft.getCurrentPosition());
            telemetry.addData("Horizontal Pos", imuOdometry.horizontalEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
