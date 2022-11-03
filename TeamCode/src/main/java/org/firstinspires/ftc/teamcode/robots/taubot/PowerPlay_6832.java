/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.robots.taubot;
//not a fruit
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Constants.LOW_BATTERY_VOLTAGE;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.taubot.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.taubot.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.VisionProviders;

import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both
 * TeleOp and Autonomous.
 */

@TeleOp(name = "PowerPlay_6832", group = "Challenge") // @Autonomous(...) is the other common choice
// @Autonomous
@Config
public class PowerPlay_6832 extends OpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    //private PoseFishin.RobotType currentBot = PoseFishin.RobotType.TomBot;

    static Robot robot;
    static Autonomous auto;
    private FtcDashboard dashboard;
    ExponentialSmoother forwardSmoother, rotateSmoother;
    public static double FORWARD_SMOOTHING_FACTOR = 0.3;
    public static double ROTATE_SMOOTHING_FACTOR = 0.25;
    DriverControls dc;
    boolean joystickDriveStarted = false;

    static public int state = 0;

    // global state
    static boolean active;
    static boolean debugTelemetryEnabled;
    private boolean initializing,  numericalDashboardEnabled, smoothingEnabled;
    static Constants.Alliance alliance;
    static Constants.Position startingPosition;
    static GameState gameState;
    static int gameStateIndex;

    private long startTime;

    // vision state
    static int visionProviderIndex;
    static boolean visionProviderFinalized;

    // loop time profile
    long lastLoopClockTime, loopTime;
    double loopAvg = 0;
    private static final double loopWeight = .1;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother, averageUpdateTimeSmoother;
    public static double AVERAGE_LOOP_TIME_SMOOTHING_FACTOR = 0.1;
    public static boolean DEFAULT_DEBUG_TELEMETRY_ENABLED = false;

    // drive train control variables
    private double pwrDamper = 1;
    private int direction = 1; // -1 to reverse direction

    // sensors/sensing-related variables
    private Orientation angles;

    // these are meant as short term testing variables, don't expect their usage
    // to be consistent across development sessions
    // private double testableDouble = robot.kpDrive;
    private double testableHeading = 0;
    private boolean testableDirection = true;

    // values associated with the buttons in the toggleAllowedGP2 method
    private boolean[] buttonSavedStates2 = new boolean[16];

    public static double RUMBLE_DURATION = 0.5;
    boolean armCalibrated = false;

    boolean debugTelemetry = false;

    // game mode configuration
    private int gameMode = 0;
    private static final int NUM_MODES = 4;
    private static final String[] GAME_MODES = { "REVERSE", "ENDGAME", "PRE-GAME", "REGULAR" };
    private boolean endGameHandled;

    public enum GameState {
        AUTONOMOUS("Autonomous", true),

        TELE_OP("Tele-Op"),
        DEMO("Demo"),
        MANUAL_DIAGNOSTIC("Manual Diagnostic"),

        CRANE_DEBUG("Crane Debug"),

        BACK_AND_FORTH("Back And Forth"),
        SQUARE("Square"),
        SQUARENORR("Square No RR"),
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

        public static GameState getGameState(int index) {
            return GameState.values()[index];
        }

        public static int getNumGameStates() {
            return GameState.values().length;
        }

        public static int indexOf(GameState gameState) {
            return Arrays.asList(GameState.values()).indexOf(gameState);
        }
    }


    // sound related configuration
    private int soundState = 0;
    private int soundID = -1;

    // auto stuff
    private double pCoeff = 0.14;
    private double dCoeff = 1.31;
    private double targetAngle = 287.25;

    private int craneArticulation = 1;

    private boolean stopAll = false;

     // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {

        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        // global state
        active = true;
        initializing = true;
        debugTelemetryEnabled = DEFAULT_DEBUG_TELEMETRY_ENABLED;
        gameState = GameState.TELE_OP;

        // timing
        lastLoopClockTime = System.nanoTime();
        loopTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);
        averageUpdateTimeSmoother = new ExponentialSmoother(AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        robot = new Robot(hardwareMap,false);



        // gamepads
        dc = new DriverControls(gamepad1,gamepad2);

        auto = new Autonomous(robot);


        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);

        forwardSmoother = new ExponentialSmoother(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother = new ExponentialSmoother(ROTATE_SMOOTHING_FACTOR);

        auto.createVisionProvider(VisionProviders.DEFAULT_PROVIDER_INDEX);
        auto.visionProvider.initializeVision(hardwareMap);
        visionProviderFinalized = true;

        debugTelemetry = true;
        if (debugTelemetry)
            configureDashboardDebug();
        else
            configureDashboardMatch();
        telemetry.update();



    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {



        //run all driver controls needed in init_loop
        dc.init_loop();

        update();


        /*
        try {
            Thread.sleep(0);
        }catch(InterruptedException e){}


         */


        //while (!robot.crane.calibrate()){} //omg no loops!
        if (!armCalibrated) armCalibrated=robot.crane.calibrate();
    }
    private void rumble() {
        gamepad1.rumble((int) (RUMBLE_DURATION * 1000));
        gamepad2.rumble((int) (RUMBLE_DURATION * 1000));
    }

    @Override
    public void start(){
        //
        // THIS SECTION EXECUTES ONCE RIGHT AFTER START IS PRESSED
        //


        lastLoopClockTime = System.nanoTime();
        startTime = System.currentTimeMillis();

        rumble();



    }

        @Override
    public void loop() {
            dc.updateStickyGamepads();
            dc.handleStateSwitch();

            if (active) {
                long currentTime = System.currentTimeMillis();
                if (!endGameHandled && gameState == PowerPlay_6832.GameState.TELE_OP && (currentTime - startTime) * 1e-3 >= 80) {
//                robot.articulate(Robot.Articulation.START_END_GAME);
                    endGameHandled = true;
                    rumble();
                }
                switch(gameState) {
                    case TELE_OP:
                        dc.joystickDrive();
                        break;
                    case DEMO:
                        //demo();
                        break;
                    case MANUAL_DIAGNOSTIC:
                        //handleManualDiagnostic();

                        break;

                    case CRANE_DEBUG:
                        //handleCraneDebug();
                        break;
                    case AUTONOMOUS:
                        //if(auto.getStateMachine(startingPosition, org.firstinspires.ftc.teamcode.robots.reachRefactor.Autonomous.Mode.SPLINE).execute())
                        //    changeGameState(FF_6832.GameState.TELE_OP);
                        // auton full
                        break;
                    case BACK_AND_FORTH:
                        //auto.backAndForth.execute();
                        break;
                    case SQUARE:
                        //auto.square.execute();
                        break;
                    case SQUARENORR:
                        //auto.squareNoRR.execute();
                        break;
                    case TURN:
                        //auto.turn.execute();
                }
            } else {
                dc.handlePregameControls();
            }

            update();

        }

    private void initialization_initSound() {
        telemetry.addData("Please wait", "Initializing Sound");
        // telemetry.update();
        soundID = hardwareMap.appContext.getResources().getIdentifier("gracious", "raw",
                hardwareMap.appContext.getPackageName());
        boolean success = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        if (success)
            soundState = 1;
        else
            soundState = 2;
    }

    private void configureDashboardDebug() {
        // Configure the dashboard.

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        // telemetry.addAction(() ->
        // Acquiring the angles is relatively expensive; we don't want
        // to do that in each of the three items that need that info, as that's
        // three times the necessary expense.
        // angles =
        // robot.imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX)

        // );

        telemetry.addLine().addData("active", () -> active);
        telemetry.addLine().addData("state", () -> state);
    }

    private void configureDashboardMatch() {
        // Configure the dashboard.

        telemetry.addLine().addData("active", () -> active).addData("state", () -> state)
                .addData("Game Mode", () -> GAME_MODES[gameMode]);

        telemetry.addLine().addData("Loop time", "%.0fms", () -> loopAvg / 1000000).addData("Loop time", "%.0fHz",
                () -> 1000000000 / loopAvg);

    }

    private void updateTiming() {
        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopClockTime;
        averageLoopTime = loopTimeSmoother.update(loopTime);
        lastLoopClockTime = loopClockTime;
    }

    private void handleTelemetry(Map<String, Object> telemetryMap, String telemetryName, TelemetryPacket packet) {
        telemetry.addLine(telemetryName);
        packet.addLine(telemetryName);

        if(robot.getVoltage() <= LOW_BATTERY_VOLTAGE) {
            telemetryMap = new LinkedHashMap<>();
            for(int i = 0; i < 20; i++) {
                telemetryMap.put(i +
                                (System.currentTimeMillis() / 500 % 2 == 0 ? "**BATTERY VOLTAGE LOW**" : "  BATTERY VOLTAGE LOW  "),
                        (System.currentTimeMillis() / 500 % 2 == 0 ? "**CHANGE BATTERY ASAP!!**" : "  CHANGE BATTERY ASAP!!  "));
            }
        }
        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            String line = Misc.formatInvariant("%s: %s", entry.getKey(), entry.getValue());
            if(numericalDashboardEnabled)
                packet.put(entry.getKey(), entry.getValue());
            else
                packet.addLine(line);
            telemetry.addLine(line);
        }

        telemetry.addLine();
        packet.addLine("");
    }

    private void update() {

        // handling dashboard changes
        forwardSmoother.setSmoothingFactor(FORWARD_SMOOTHING_FACTOR);
        rotateSmoother.setSmoothingFactor(ROTATE_SMOOTHING_FACTOR);

       Pose2d target = robot.field.targetCoordinate;
       Pose2d current = robot.field.poseToCoordinates(robot.driveTrain.getPoseEstimate());
       Pose2d current2 = robot.driveTrain.getPoseEstimate();
        telemetry.addLine("target X" + target.getX());
        telemetry.addLine("target Y" + target.getY());

        telemetry.addLine("current_coordinate X" + current.getX());
        telemetry.addLine("current_coordinate Y" + current.getY());

        telemetry.addLine("current_Pose X" + current2.getX());
        telemetry.addLine("current_Pose Y" + current2.getY());
        TelemetryPacket packet = new TelemetryPacket();

        long updateStartTime = System.nanoTime();
        robot.update(packet.fieldOverlay());
        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);

        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();
        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
            opModeTelemetryMap.put("Smoothing Enabled", smoothingEnabled);
        }
        opModeTelemetryMap.put("Average Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        opModeTelemetryMap.put("Last Loop Time", Misc.formatInvariant("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));
        if(debugTelemetryEnabled) {
            opModeTelemetryMap.put("Average Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (averageUpdateTime * 1e-6), (int) (1 / (averageUpdateTime * 1e-9))));
            opModeTelemetryMap.put("Last Robot Update Time", Misc.formatInvariant("%d ms (%d hz)", (int) (updateTime * 1e-6), (int) (1 / (updateTime * 1e-9))));
        }

        //here we can add telemetry specific to certain gameStates
        switch(gameState) {
            case TELE_OP:
                //opModeTelemetryMap.put("Double Duck", robot.isDoubleDuckEnabled());
                break;
            case MANUAL_DIAGNOSTIC:
                //opModeTelemetryMap.put("Diagnostic Step", diagnosticStep);
                break;
        }

        handleTelemetry(opModeTelemetryMap,  Misc.formatInvariant("(%d): %s", gameStateIndex, gameState.getName()), packet);


        // handling subsystem telemetry
        for(TelemetryProvider telemetryProvider: robot.subsystems)
            handleTelemetry(telemetryProvider.getTelemetry(debugTelemetryEnabled), telemetryProvider.getTelemetryName(), packet);

        handleTelemetry(robot.getTelemetry(debugTelemetryEnabled), robot.getTelemetryName(), packet);

        // handling vision telemetry
        Map<String, Object> visionTelemetryMap = auto.visionProvider.getTelemetry(debugTelemetryEnabled);
        visionTelemetryMap.put("Backend",
                Misc.formatInvariant("%s (%s)",
                        VisionProviders.VISION_PROVIDERS[visionProviderIndex].getSimpleName(),
                        visionProviderFinalized ?
                                "finalized" :
                                System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : "  NOT FINALIZED  "
                )
        );


        handleTelemetry(visionTelemetryMap, auto.visionProvider.getTelemetryName(), packet);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        //if(!initializing)
            //dashboard.sendImage(robot.getBitmap());

        updateTiming();
    }

}
