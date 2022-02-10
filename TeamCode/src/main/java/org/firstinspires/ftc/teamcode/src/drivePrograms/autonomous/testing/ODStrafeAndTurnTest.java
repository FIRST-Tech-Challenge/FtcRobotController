package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.robotAttachments.navigation.navigationWarnings.DistanceTimeoutWarning;
import org.firstinspires.ftc.teamcode.src.utills.opModeTemplate.AutonomousTemplate;

@Disabled
@TeleOp(name = "OD Strafe And Turn Test")
public class ODStrafeAndTurnTest extends AutonomousTemplate {

    @Override
    public void opModeMain() throws InterruptedException {
        initAll();
        driveSystem.setTurnWhileStrafe(true);

        final double sideLength = 40;
        final double halfSideLength = sideLength / 2.0D;

        waitForStart();
        gps.setPos(0, 0, 0);

        driveSystem.moveToPosition(halfSideLength, halfSideLength, 90, 1, new DistanceTimeoutWarning(1000));


        driveSystem.moveToPosition(halfSideLength, -halfSideLength, 180, 1, new DistanceTimeoutWarning(1000));


        driveSystem.moveToPosition(-halfSideLength, -halfSideLength, 270, 1, new DistanceTimeoutWarning(1000));


        driveSystem.moveToPosition(-halfSideLength, halfSideLength, 360, 1, new DistanceTimeoutWarning(1000));


        driveSystem.moveToPosition(0, 0, 0, 1, new DistanceTimeoutWarning(1000));


    }
}
