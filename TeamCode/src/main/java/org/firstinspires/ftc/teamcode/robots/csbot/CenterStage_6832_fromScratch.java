package org.firstinspires.ftc.teamcode.robots.csbot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robots.csbot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.csbot.util.Constants;

import java.util.Arrays;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "0 CenterStage_6832", group = "Challenge") // @Autonomous(...) is the other common choice
@Config(value = "AA_CS_6832")
public class CenterStage_6832_fromScratch extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //COMPONENTS
    static Robot robot;
    static Autonomous auto;
    private FtcDashboard dashboard;
    DriverControls dc;

    //GLOBAL STATES
    public static boolean active;
    public static boolean debugTelemetryEnabled;
    private boolean initializing;

    //GAMESTATES
    public enum GameState {
        //MAIN OPMODES
        AUTONOMOUS("Autonomous", true),
        TELE_OP("Tele-Op"),

        //TEST & TEMP MODES
        TEST("Test"),
        DEMO("Demo"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic"),
        SQUARE("Square"),
        TURN("Turn");

        private final String name;
        private final boolean autonomous;

        GameState(String name, boolean autonomous) {
            this.name = name;
            this.autonomous = autonomous;
        }

        GameState(String name) {
            this(name, false);
        }

        public String getName() { return name; }

        public boolean isAutonomous() { return autonomous; }

    }
    public static GameState gameState = GameState.AUTONOMOUS;


    //CONSTANTS FOR GAME
    static Constants.Alliance alliance;
    static Constants.Position startingPosition;
    long startTime;


    //CONSTANTS
    public static Constants.Position origin;


    //LIVE DATA
    private double averageLoopTime;
    private double averageVoltage;


    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {
            if(active)
            {

            }
            else
            {
                dc.handlePregameControls();
            }
    }
}


