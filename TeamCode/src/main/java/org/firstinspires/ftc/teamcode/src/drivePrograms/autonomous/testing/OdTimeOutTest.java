package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.utills.Executable;

@Disabled
@Autonomous(name = "OdTimeOut")
public class OdTimeOutTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 116, 180);
        waitForStart();

        try {
            driveSystem.moveToPositionWithDistanceTimeOut(150, 116, 1, true, 500);
        } catch (OdometryMovementException e) {
            telemetry.addData("Distance Time Out", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

        try {
            driveSystem.moveToPositionWithTimeOut(100000, 100000, 1, false, 0.1);
        } catch (OdometryMovementException e) {
            telemetry.addData("Standard Time Out", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

        try {
            driveSystem.moveToPositionWithVoltageSpike(odometry.returnRelativeXPosition(), odometry.returnRelativeYPosition() + 100, 1, false);
        } catch (OdometryMovementException e) {
            telemetry.addData("Voltage Spike", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }

        try {
            Executable<Boolean> b = () -> {
                return true;
            };
            driveSystem.moveToPositionWithCallBack(100000, 100000, 1, b, false);
        } catch (OdometryMovementException e) {
            telemetry.addData("Callback Error", "Yay");
            telemetry.update();
            Thread.sleep(1000);
        }


    }
}
