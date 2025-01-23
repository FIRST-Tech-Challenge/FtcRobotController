package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;


@Config
public class GlobalData {
    public static boolean hasGamePiece = false;
    public static boolean inAutonomous = false;
    public static float currentTime = 0;
    public static float lastTime = 0;
    public static float deltaTime = 0;
    public static boolean AllianceColor = true; // true = blue, false = red
    public static boolean wasInAutonomous = false; // TODO => make this true in the end of all autos
    public static boolean inCompetition = false; // TODO => in the competition make this true
    public static boolean allowDriveByAprilTagsAssistAndAutoDrive = true;
    public static boolean allowDriveByObjectsAssistAndAutoDrive = true;
    public static boolean assistActive = false;
    public static boolean inAutoDrive = false;
    public static boolean firstCycleInTeleop = true;
    public static boolean usingDashBoardGampad = false;
}