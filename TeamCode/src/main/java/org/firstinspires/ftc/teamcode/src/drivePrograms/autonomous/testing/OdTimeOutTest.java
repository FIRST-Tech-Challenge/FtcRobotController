package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.OdometryMovementException;
import org.firstinspires.ftc.teamcode.src.utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.utills.MiscUtills;


@Autonomous(name = "OdTimeOut")
public class OdTimeOutTest extends AutonomousTemplate {
    @Override
    public void opModeMain() throws InterruptedException {
        this.initAll();
        odometry.setPosition(0, 116, 180);
        waitForStart();
        try {
            driveSystem.moveToPositionWithTimeOut(150, 116, 1, true, 500);
        } catch (OdometryMovementException e) {
            driveSystem.strafeAtAngle(0, .5);
            Thread.sleep(1000);
            while (!isStopRequested()) {
                telemetry.addData("this", "happened");
                telemetry.addData(MiscUtills.getStackTraceAsString(e), "");
                telemetry.update();
                //driveSystem.moveToPosition(0,116,1);
            }
        }


    }
}
