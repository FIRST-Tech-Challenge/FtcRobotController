package org.firstinspires.ftc.teamcode;

import android.content.SharedPreferences;
import android.preference.PreferenceManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.Random;

@Autonomous(name="PowerPlayAuton", group="Linear OpMode")
public class PowerPlayAuton extends LinearOpMode {

    private RobotManager robotManager;
    private ElapsedTime elapsedTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() {
        initSharedPreferences();
        robotManager = new RobotManager(hardwareMap, gamepad1, gamepad2, PowerPlayAuton.navigationPath,
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

//        telemetry.addData("Path name", navigationPath.toString());
//        telemetry.addData("AutonomousPaths.CYCLE_HIGH.size()", Navigation.AutonomousPaths.CYCLE_HIGH.size());
//        telemetry.addData("PowerPlayAuton.navigationPath.size()", PowerPlayAuton.navigationPath.size());
//        telemetry.addData("robotManager.navigation.path.size()", robotManager.navigation.path.size());
//        telemetry.update();

        // Transform the path and add the parking location based on the result of cv
        robotManager.navigation.configurePath(startingSide, parkingPosition);

        waitMilliseconds(PowerPlayAuton.waitTime);

        robotManager.runAutonPath();

        while (opModeIsActive()) {}
    }

    private void waitMilliseconds(long ms) {
        double start_time = elapsedTime.time();
        while (opModeIsActive() && elapsedTime.time() - start_time < ms) {}
    }

    // ANDROID SHARED PREFERENCES
    // ==========================

    // Adapted from https://github.com/ver09934/twentytwenty/blob/ian-dev/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/SkystoneAuton.java

    private static SharedPreferences sharedPrefs;

    private static int waitTime = 0;
    private static Navigation.MovementMode movementMode;
    private static RobotManager.StartingSide startingSide;
    private static RobotManager.AllianceColor allianceColor;
    private static ArrayList<Position> navigationPath;

    public void initSharedPreferences() {
        sharedPrefs = PreferenceManager.getDefaultSharedPreferences(this.hardwareMap.appContext);

        String movementMode = sharedPrefs.getString("movement_mode", "ERROR");
        String waitTime = sharedPrefs.getString("wait_time", "ERROR");
        String startingSide = sharedPrefs.getString("starting_side", "ERROR");
        String allianceColor = sharedPrefs.getString("alliance_color", "ERROR");
        String autonMode = sharedPrefs.getString("auton_type", "ERROR");

        telemetry.addData("Movement mode", movementMode);
        telemetry.addData("Wait time", waitTime);
        telemetry.addData("Auton mode", autonMode);
        telemetry.addData("Starting side", startingSide);
        telemetry.addData("Alliance color", allianceColor);

        System.out.println("Movement mode "+ movementMode);
        System.out.println("Wait time "+ waitTime);
        System.out.println("Auton mode "+ autonMode);
        System.out.println("Starting side "+ startingSide);
        System.out.println("Alliance color "+ allianceColor);


        switch (movementMode) {
            case "STRAFE":
                PowerPlayAuton.movementMode = Navigation.MovementMode.STRAFE;
                break;
            case "FORWARD_ONLY":
                PowerPlayAuton.movementMode = Navigation.MovementMode.FORWARD_ONLY;
                break;
        }

        switch (waitTime) {
            case "0_SECONDS":
                PowerPlayAuton.waitTime = 0;
                break;
            case "5_SECONDS":
                PowerPlayAuton.waitTime = 5;
                break;
            case "10_SECONDS":
                PowerPlayAuton.waitTime = 10;
                break;
            case "15_SECONDS":
                PowerPlayAuton.waitTime = 15;
                break;
            case "20_SECONDS":
                PowerPlayAuton.waitTime = 20;
                break;
        }

        switch(startingSide) {
            case "LEFT":
                PowerPlayAuton.startingSide = RobotManager.StartingSide.LEFT;
                break;

            case "RIGHT":
                PowerPlayAuton.startingSide = RobotManager.StartingSide.RIGHT;
                break;
        }

        if (allianceColor.equals("BLUE")) {
            PowerPlayAuton.allianceColor = RobotManager.AllianceColor.BLUE;
        }
        else if (allianceColor.equals("RED")) {
            PowerPlayAuton.allianceColor = RobotManager.AllianceColor.RED;
        }

        switch (autonMode) {
//            case "SMALL":
//                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.SMALL.clone();
//                break;
//            case "MEDIUM":
//                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.MEDIUM.clone();
//                break;
//            case "LARGE":
//                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.LARGE.clone();
//                break;
//            case "PARK_ONLY":
//                PowerPlayAuton.navigationPath = (ArrayList<Position>) AutonomousPaths.PARK_ONLY.clone();
            case "CYCLE_HIGH":
                PowerPlayAuton.navigationPath = (ArrayList<Position>) Navigation.AutonomousPaths.CYCLE_HIGH.clone();
                break;
            case "PARK_ONLY":
                PowerPlayAuton.navigationPath = new ArrayList<>();
                break;
        }
    }
}
