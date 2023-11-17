package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Random;

@Autonomous(name="CenterStageAuton", group="Linear OpMode")
public class CenterStageAuton extends LinearOpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, CenterStageTeleop.navigationPath,
                PowerPlayAuton.allianceColor, PowerPlayAuton.startingSide,
                PowerPlayAuton.movementMode, telemetry, elapsedTime);
        IMUPositioning.Initialize(this);
        robotManager.computerVision.initialize();
        robotManager.closeClaw();

        // Repeatedly run CV
        RobotManager.ParkingPosition parkingPosition = RobotManager.ParkingPosition.LEFT;
        while (!isStarted() && !isStopRequested()) {
            parkingPosition = robotManager.computerVision.getParkingPosition();
            waitMilliseconds(20);
        }
    }
}
