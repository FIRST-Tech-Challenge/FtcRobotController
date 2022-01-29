package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.MovementException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationErrors.DistanceTimeoutError;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;

@Disabled
@Autonomous(name = "OdTimeOut")
public class OdTimeOutTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPos(0, 0, 0);
        waitForStart();
        driveSystem.debugOn();
        try {
            driveSystem.moveToPosition(50, 0, odometry.getRot(), 1, new DistanceTimeoutError(500));
        } catch (MovementException e) {
            telemetry.addData("Distance Time Out", "Yay");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) Thread.sleep(20);
        }


    }
}
