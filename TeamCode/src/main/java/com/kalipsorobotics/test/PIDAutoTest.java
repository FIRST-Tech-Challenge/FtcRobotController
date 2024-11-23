package com.kalipsorobotics.test;

import com.kalipsorobotics.actions.MecanumRobotAction;
import com.kalipsorobotics.actions.MoveRobotStraightInchesAction;
import com.kalipsorobotics.actions.TurnRobotAction;
import com.kalipsorobotics.localization.Odometry;
import com.kalipsorobotics.modules.DriveTrain;
import com.kalipsorobotics.utilities.OpModeUtilities;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="PIDAutoTest")
public class PIDAutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        OpModeUtilities opModeUtilities = new OpModeUtilities(hardwareMap, this, telemetry);
        DriveTrain driveTrain = new DriveTrain(opModeUtilities);
        Odometry odometry = new Odometry(driveTrain, opModeUtilities, 0, 0, 0);

        MoveRobotStraightInchesAction straightInchesAction = new MoveRobotStraightInchesAction(24, driveTrain, odometry);
        MecanumRobotAction mecanumAction = new MecanumRobotAction(12, driveTrain, odometry);
        TurnRobotAction turnAction = new TurnRobotAction(45, driveTrain, odometry);
    }
}
