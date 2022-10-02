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
package org.firstinspires.ftc.teamcode.robots.LilVirani;

import static org.firstinspires.ftc.teamcode.robots.LilVirani.util.Constants.LOW_BATTERY_VOLTAGE;
import static org.firstinspires.ftc.teamcode.util.utilMethods.notdeadzone;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.internal.system.Misc;
import org.firstinspires.ftc.teamcode.robots.LilVirani.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.LilVirani.util.Constants;
import org.firstinspires.ftc.teamcode.robots.LilVirani.util.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.LilVirani.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.vision.SkystoneTargetInfo;

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

    private Robot robot;
    private FtcDashboard dashboard;

    private boolean joystickDriveStarted = false;

    static public int state = 0;

    // global state
    private boolean active, initializing, debugTelemetryEnabled, numericalDashboardEnabled, smoothingEnabled, antiTippingEnabled;
    private Constants.Alliance alliance;
    private Constants.Position startingPosition;
    private GameState gameState;
    private int gameStateIndex;
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private long startTime;

    // vision state
    private int visionProviderIndex;
    private boolean visionProviderFinalized;

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
    private double pwrFwd = 0;
    private double pwrStf = 0;
    private double pwrRot = 0;
    private double pwrFwdL = 0;
    private double pwrStfL = 0;
    private double pwrFwdR = 0;
    private double pwrStfR = 0;
    private double beaterDamper = .75;
    private boolean enableTank = false;
    private boolean bypassJoysticks = false;
    private long damperTimer = 0;
    private int direction = 1; // -1 to reverse direction
    private int currTarget = 0;

    // sensors/sensing-related variables
    private Orientation angles;

    // these are meant as short term testing variables, don't expect their usage
    // to be consistent across development sessions
    // private double testableDouble = robot.kpDrive;
    private double testableHeading = 0;
    private boolean testableDirection = true;

    // values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[16];
    private int a = 0; // lower glyph lift
    private int b = 1; // toggle grip/release on glyph
    private int x = 2; // no function
    private int y = 3; // raise glyph lift
    private int dpad_down = 4; // enable/disable ftcdash telemetry
    private int dpad_up = 5; // vision init/de-init
    private int dpad_left = 6; // vision provider switch
    private int dpad_right = 7; // switch viewpoint
    private int left_bumper = 8; // increment state down (always)
    private int right_bumper = 9; // increment state up (always)
    private int startBtn = 10; // toggle active (always)
    private int left_trigger = 11; // vision detection
    private int right_trigger = 12;
    private int back_button = 13;
    private int left_stick_button = 14;
    private int right_stick_button = 15; // sound player

    // values associated with the buttons in the toggleAllowedGP2 method
    private boolean[] buttonSavedStates2 = new boolean[16];

    public static double RUMBLE_DURATION = 0.5;

    boolean debugTelemetry = false;

    int stateLatched = -1;
    int stateIntake = -1;
    int stateDelatch = -1;
    boolean isIntakeClosed = true;
    boolean isHooked = false;
    boolean enableHookSensors = false;
    boolean calibrateFirstHalfDone = false;

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
    private SkystoneTargetInfo initGoldPosTest;
    private double pCoeff = 0.14;
    private double dCoeff = 1.31;
    private double targetAngle = 287.25;

    private int craneArticulation = 1;

    private boolean stopAll = false;

    Telemetry dummyT = new Telemetry() {
        @Override
        public Item addData(String caption, String format, Object... args) {
            return null;
        }

        @Override
        public Item addData(String caption, Object value) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, Func<T> valueProducer) {
            return null;
        }

        @Override
        public <T> Item addData(String caption, String format, Func<T> valueProducer) {
            return null;
        }

        @Override
        public boolean removeItem(Item item) {
            return false;
        }

        @Override
        public void clear() {

        }

        @Override
        public void clearAll() {

        }

        @Override
        public Object addAction(Runnable action) {
            return null;
        }

        @Override
        public boolean removeAction(Object token) {
            return false;
        }

        @Override
        public boolean update() {
            return false;
        }

        @Override
        public Line addLine() {
            return null;
        }

        @Override
        public Line addLine(String lineCaption) {
            return null;
        }

        @Override
        public boolean removeLine(Line line) {
            return false;
        }

        @Override
        public boolean isAutoClear() {
            return false;
        }

        @Override
        public void setAutoClear(boolean autoClear) {

        }

        @Override
        public int getMsTransmissionInterval() {
            return 0;
        }

        @Override
        public void setMsTransmissionInterval(int msTransmissionInterval) {

        }

        @Override
        public String getItemSeparator() {
            return null;
        }

        @Override
        public void setItemSeparator(String itemSeparator) {

        }

        @Override
        public String getCaptionValueSeparator() {
            return null;
        }

        @Override
        public void setCaptionValueSeparator(String captionValueSeparator) {

        }

        @Override
        public void setDisplayFormat(DisplayFormat displayFormat) {

        }

        @Override
        public Log log() {
            return null;
        }

        @Override
        public void speak(String text) {
        }

        @Override
        public void speak(String text, String languageCode, String countryCode) {
        }

    };

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
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        //auto = new Autonomous(robot, dummyT, gamepad1);
        dashboard = FtcDashboard.getInstance();
        dashboard.setTelemetryTransmissionInterval(25);
        telemetry.setMsTransmissionInterval(25);

        debugTelemetry = gamepad1.right_trigger > .3;
        debugTelemetry = true;
        if (debugTelemetry)
            configureDashboardDebug();
        else
            configureDashboardMatch();
        telemetry.update();

    }
    private void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                gameStateIndex += 1;

            if(gameStateIndex < 0)
                gameStateIndex = GameState.getNumGameStates() - 1;
            gameStateIndex %= GameState.getNumGameStates();
            gameState = GameState.getGameState(gameStateIndex);
        }

        if (stickyGamepad1.back || stickyGamepad2.back)
            active = !active;
    }

    private void handleVisionProviderSwitch() {

    }

    private void handlePregameControls() {
        Constants.Position previousStartingPosition = startingPosition;
        if(stickyGamepad1.x || stickyGamepad2.x) {
            alliance = Constants.Alliance.BLUE;
            startingPosition = Constants.Position.START_BLUE_UP;
        }
        if(stickyGamepad1.a || stickyGamepad2.a) {
            alliance = Constants.Alliance.BLUE;
            startingPosition = Constants.Position.START_BLUE_DOWN;
        }
        if(stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
            startingPosition = Constants.Position.START_RED_DOWN;
        }
        if(stickyGamepad1.y || stickyGamepad2.y) {
            alliance = Constants.Alliance.RED;
            startingPosition = Constants.Position.START_RED_UP;
        }
        if(previousStartingPosition != startingPosition) {
            //todo these lines need to be enabled once we build the drivetrain and auton routines for powerplay
            //robot.driveTrain.setPoseEstimate(startingPosition.getPose());
            //auto.build(startingPosition);
        }
/*
        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;
        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down)
            if (robot.crane.shoulderInitialized)
                robot.articulate(Robot.Articulation.START_DOWN); //stow crane to the starting position
            else
                robot.crane.configureShoulder(); //setup the shoulder - do this only when the
        if(stickyGamepad1.left_trigger || stickyGamepad2.left_trigger)
            numericalDashboardEnabled = !numericalDashboardEnabled;
        if(stickyGamepad1.right_trigger || stickyGamepad2.right_trigger)
            antiTippingEnabled = !antiTippingEnabled;
        if(stickyGamepad1.right_stick_button || stickyGamepad2.right_stick_button)
            smoothingEnabled = !smoothingEnabled;
        if(stickyGamepad1.left_stick_button || stickyGamepad2.left_stick_button)
            robot.crane.articulate(Crane.Articulation.TEST_INIT);

 */
        // we can do very basic driving to get to calibration position
        // turret and drive controls on gamepad1 only since we don't always have 2 pads
        // for auton testing

        // this test suppresses pregame driving while a calibration articulation is
        // active
        // red alliance
        if (toggleAllowed(gamepad1.b, b, 1)) {
            calibrateInitStageMethod(false);
        }

        // blue alliance
        if (toggleAllowed(gamepad1.x, x, 1)) {
            calibrateInitStageMethod(true);
        }
        //resets the headings of the turret and chassis to initial values - press only after careful alignment perpendicular to alliance wall



    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {

        handleStateSwitch();
        handleVisionProviderSwitch();
        robot.handleTankDrive(gamepad1,gamepad2);
        Pose2d currentPose = robot.driveTrain.currentPose;
        telemetry.addLine("Current X " + currentPose.getX() );
        telemetry.addLine("Current Y " + currentPose.getY() );
        telemetry.addLine("Current Heading " + currentPose.getHeading() );

        update();


        /*
        try {
            Thread.sleep(0);
        }catch(InterruptedException e){}


         */



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

            handleStateSwitch();

            if (active) {
                long currentTime = System.currentTimeMillis();
                if (!endGameHandled && gameState == PowerPlay_6832.GameState.TELE_OP && (currentTime - startTime) * 1e-3 >= 80) {
//                robot.articulate(Robot.Articulation.START_END_GAME);
                    endGameHandled = true;
                    rumble();
                }
                switch(gameState) {
                    case TELE_OP:
                        joystickDrive();
                        break;
                    case DEMO:
                        demo();
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
                handlePregameControls();
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

    private void demo() {

    }

    int reverse = 1;

    private void joystickDrive() {


        // robot.crane.extendToTowerHeight(0.25, Config.currentTowerHeight);

        reverse = -1;
        pwrDamper = .70;

        pwrFwd = 0;
        pwrRot = 0;

        if (notdeadzone(gamepad1.left_stick_y))
            pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        if (notdeadzone(gamepad1.right_stick_x))
            pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;



        // old mecanum controls
        // pwrFwdL = direction * pwrDamper * gamepad1.left_stick_y;
        // pwrStfL = direction * pwrDamper * gamepad1.left_stick_x;
        //
        // pwrFwdR = direction * pwrDamper * gamepad1.right_stick_y;
        // pwrStfR = direction * pwrDamper * gamepad1.right_stick_x;

        // gamepad1 controls

        // trigger retractFromTower articulation
        // if(toggleAllowed(gamepad1.a,a,1)){
        // robot.articulate(PoseSkystone.Articulation.retractFromTower);
        // }

    }

    private void joystickDrivePregameMode() {
        // robot.setAutonSingleStep(true); //single step through articulations having to
        // do with deploying


        reverse = -1;

        pwrDamper = .70;

        // drive joysticks
        pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;


        // turret controls - this is on gamepad2 in teleop - but on gamepad 1 for
        // prematch setup

    }


    public void calibrateInitStageMethod(boolean isBlue) {



    }

    private void turnTest() {

    }

    // the method that controls the main state of the robot; must be called in the
    // main loop outside of the main switch
    private void stateSwitch() {
        if (!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper, 1)) {

                state--;
                if (state < 0) {
                    state = 10;
                }
                active = false;
            }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper, 1)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                active = false;
            }

        }

        if (toggleAllowed(gamepad1.start, startBtn, 1)) {
            active = !active;
        }
    }

    // checks to see if a specific button should allow a toggle at any given time;
    // needs a rework
    private boolean toggleAllowed(boolean button, int buttonIndex, int gpId) {
        if (button) {
            if (gpId == 1) {
                if (!buttonSavedStates[buttonIndex]) { // we just pushed the button, and when we last looked at it, it
                    // was not pressed
                    buttonSavedStates[buttonIndex] = true;
                    return true;
                } else { // the button is pressed, but it was last time too - so ignore

                    return false;
                }
            } else {
                if (!buttonSavedStates2[buttonIndex]) { // we just pushed the button, and when we last looked at it, it
                    // was not pressed
                    buttonSavedStates2[buttonIndex] = true;
                    return true;
                } else { // the button is pressed, but it was last time too - so ignore

                    return false;
                }
            }
        }
        if (gpId == 1)
            buttonSavedStates[buttonIndex] = false; // not pressed, so remember that it is not
        else
            buttonSavedStates2[buttonIndex] = false;
        return false; // not pressed

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

    private int servoTest = 1005;

    private void servoTest() {
        // robot.ledSystem.movement.setPosition(Conversions.servoNormalize(servoTest));
        if (toggleAllowed(gamepad1.a, a, 1))
            servoTest -= 10;
        else if (toggleAllowed(gamepad1.y, y, 1))
            servoTest += 10;
        telemetry.addData("Pulse width", servoTest);
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

        //todo use this once we have stuff moved into drivetrain
        // if(robot.driveTrain.getVoltage() <= LOW_BATTERY_VOLTAGE) {
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
        stickyGamepad1.update();
        stickyGamepad2.update();

        // handling dashboard changes
        //forwardSmoother.setSmoothingFactor(FORWARD_SMOOTHING_FACTOR);
        //rotateSmoother.setSmoothingFactor(ROTATE_SMOOTHING_FACTOR);

        TelemetryPacket packet = new TelemetryPacket();

        long updateStartTime = System.nanoTime();
        //robot.update(packet.fieldOverlay());
        long updateTime = (System.nanoTime() - updateStartTime);
        double averageUpdateTime = averageUpdateTimeSmoother.update(updateTime);

        Map<String, Object> opModeTelemetryMap = new LinkedHashMap<>();
        // handling op mode telemetry
        opModeTelemetryMap.put("Active", active);
        if(initializing) {
            opModeTelemetryMap.put("Starting Position", startingPosition);
            opModeTelemetryMap.put("Anti-Tipping Enabled", antiTippingEnabled);
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
        //todo renable once we put stuff into refactored subsystems
        // handling subsystem telemetry
        /*
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
        */

        //handleTelemetry(visionTelemetryMap, auto.visionProvider.getTelemetryName(), packet);
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

        //if(!initializing)
            //dashboard.sendImage(robot.getBitmap());

        updateTiming();
    }

}
