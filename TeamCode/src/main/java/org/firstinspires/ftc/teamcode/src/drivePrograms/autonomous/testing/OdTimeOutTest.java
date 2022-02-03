package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.DistanceTimeoutException;
import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationExceptions.MovementException;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

@Disabled
@Autonomous(name = "OdTimeOut")
public class OdTimeOutTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        gps.setPos(0, 0, 0);
        waitForStart();
        driveSystem.debugOn();
        try {
            driveSystem.moveToPosition(50, 0, gps.getRot(), 1, new DistanceTimeoutException(500));
        } catch (MovementException e) {
            telemetry.addData("Distance Time Out", "Yay");
            telemetry.update();
            while (opModeIsActive() && !isStopRequested()) Thread.sleep(20);
        }


    }
}
