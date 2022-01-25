package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "OdTimeOut")
public class OdTimeOutTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 0, 0);
        waitForStart();
        driveSystem.debugOn();
        try {
            driveSystem.moveToPositionWithDistanceTimeOut(50, 0, odometry.returnOrientation(), 1, 500);
        } catch (OdometryMovementException e) {
            telemetry.addData("Distance Time Out", "Yay");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) ;
        }
        /*
        try {
            driveSystem.moveToPositionWithTimeOut(100000, 100000, 1, 0.1);
        } catch (OdometryMovementException e) {
            telemetry.addData("Standard Time Out", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

        try {
            driveSystem.moveToPositionWithVoltageSpike(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition() + 100, 1);
        } catch (OdometryMovementException e) {
            telemetry.addData("Voltage Spike", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

        try {
            Executable<Boolean> b = () -> {
                return true;
            };
            driveSystem.moveToPosition(100000, 100000, odometry.returnOrientation(), 1, b);
        } catch (OdometryMovementException e) {
            telemetry.addData("Callback Error", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

         */


    }
}
