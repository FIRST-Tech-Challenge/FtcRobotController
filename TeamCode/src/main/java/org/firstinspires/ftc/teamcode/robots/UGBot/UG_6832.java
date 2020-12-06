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
package org.firstinspires.ftc.teamcode.robots.UGBot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.vision.SkystoneTargetInfo;
import org.firstinspires.ftc.teamcode.vision.StonePos;
import org.opencv.core.Mat;

import static org.firstinspires.ftc.teamcode.util.Conversions.nearZero;
import static org.firstinspires.ftc.teamcode.util.Conversions.nextCardinal;
import static org.firstinspires.ftc.teamcode.util.Conversions.notdeadzone;
import static org.firstinspires.ftc.teamcode.util.Conversions.servoNormalize;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both
 * TeleOp and Autonomous.
 */

@TeleOp(name = "UltimateGoal_6832", group = "Challenge") // @Autonomous(...) is the other common choice
// @Autonomous
@Config
public class UG_6832 extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();

    private PoseUG.RobotType currentBot = PoseUG.RobotType.TomBot;

    private PoseUG robot;

    private Autonomous auto;

    private boolean active = true;
    private boolean joystickDriveStarted = false;

    static public int state = 0;

    // loop time profile
    long lastLoopClockTime;
    double loopAvg = 0;
    private static final double loopWeight = .1;

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

    // sound related configuration
    private int soundState = 0;
    private int soundID = -1;

    // auto stuff
    private StackHeight initStackHeightTest;
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

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initializing " + currentBot + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        robot = new PoseUG(currentBot);
        robot.init(this.hardwareMap);

        auto = new Autonomous(robot, dummyT, gamepad1);

        debugTelemetry = gamepad1.right_trigger > .3;
        debugTelemetry = true;
        if (debugTelemetry)
            configureDashboardDebug();
        else
            configureDashboardMatch();
        telemetry.update();

        // waitForStart();
        // this is commented out but left here to document that we are still doing the
        // functions that waitForStart() normally does, but needed to customize it.

        robot.resetMotors(true);
        auto.visionProviderFinalized = false;

        while (!isStarted()) { // Wait for the game to start (driver presses PLAY)


            synchronized (this) {
                try {
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }

            stateSwitch();

            if (active) {

                // we can do very basic driving to get to calibration position
                // turret and drive controls on gamepad1 only since we don't always have 2 pads
                // for auton testing

                // this test suppresses pregame driving while a calibration articulation is
                // active
                if (robot.articulation == PoseUG.Articulation.manual)
                    joystickDrivePregameMode();

                // old calibraton sequence todo-
//                if (toggleAllowed(gamepad1.b, b, 1)) {
//                    calibrateInitStageMethod(false);
//                }
//
//                // blue alliance
//                if (toggleAllowed(gamepad1.x, x, 1)) {
//                    calibrateInitStageMethod(true);
//                }
                //resets the headings of the turret and chassis to initial values - press only after careful alignment perpendicular to alliance wall
                if (toggleAllowed(gamepad1.y, y, 1)) {
                    robot.setHeadingAlliance();
                }

                if (toggleAllowed(gamepad1.a, a, 1)) {
                    robot.setHeadingBase(90.0);
                }

            }

            else { // if inactive we are in configuration mode

                if(auto.visionProviderFinalized)
                    auto.sample();

                if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_left, dpad_left, 1)) {
                    auto.visionProviderState = (auto.visionProviderState + 1) % auto.visionProviders.length; // switch
                                                                                                             // vision
                                                                                                             // provider
                }
                if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up, 1)) {
                    auto.initVisionProvider(); // this is blocking
                } else if (auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_up, dpad_up, 1)) {
                    auto.deinitVisionProvider(); // also blocking, but should be very quick
                }
                if (!auto.visionProviderFinalized && toggleAllowed(gamepad1.dpad_down, dpad_down, 1)) {
                    auto.enableTelemetry = !auto.enableTelemetry; // enable/disable FtcDashboard telemetry
                    // CenterOfGravityCalculator.drawRobotDiagram =
                    // !CenterOfGravityCalculator.drawRobotDiagram;
                }
                if (auto.visionProviderFinalized && gamepad1.left_trigger > 0.3) {
                    StackHeight sp = auto.vp.detect();
                    if (sp != StackHeight.NONE_FOUND)
                        initStackHeightTest = sp;
                    telemetry.addData("Vision", "Prep detection: %s%s", initStackHeightTest,
                            sp == StackHeight.NONE_FOUND ? " (NONE_FOUND)" : "");
                }

                if (soundState == 0 && toggleAllowed(gamepad1.right_stick_button, right_stick_button, 1)) {
                    initialization_initSound();
                }

                telemetry.addData("Vision", "Backend: %s (%s)",
                        auto.visionProviders[auto.visionProviderState].getSimpleName(),
                        auto.visionProviderFinalized ? "finalized"
                                : System.currentTimeMillis() / 500 % 2 == 0 ? "**NOT FINALIZED**" : " NOT FINALIZED ");
                telemetry.addData("Vision", "FtcDashboard Telemetry: %s",
                        auto.enableTelemetry ? "Enabled" : "Disabled");
                telemetry.addData("Vision", "Viewpoint: %s", auto.viewpoint);
                telemetry.addData("Status", "Initialized");
                telemetry.addData("Status", "Auto Delay: " + Integer.toString((int) auto.autoDelay) + "seconds");

            }
            telemetry.update();

            robot.ledSystem.setColor(LEDSystem.Color.GAME_OVER);

            robot.updateSensors(active);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        } // end of stuff that happens during Init, but before Start

        //
        // THIS SECTION EXECUTES ONCE RIGHT AFTER START IS PRESSED
        //

        if (auto.vp == null) {
            auto.initDummyVisionProvider(); // this is blocking
        }

        auto.vp.reset();

        robot.launcher.restart(.4, .5);

        lastLoopClockTime = System.nanoTime();

        //
        // END OF SECTION THAT EXECUTES ONCE RIGHT AFTER START IS PRESSED
        //

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            stateSwitch();
            if (active) {
                switch (state) {
                    case 0: // auton full
                        joystickDrive();
                        break;
                    case 1: // teleop
                        joystickDrive();
                        break;
                    case 2:
                        robot.launcher.alignGripperForwardFacing();
                        break;
                    case 4:
                        break;
                    case 6:
                        demo();
                        break;
                    case 7:
                        robot.driveIMUDistanceWithReset(.6,robot.getHeading(),true,.470);
                        break;
                    case 8:
                        demo();
                        break;
                    case 9:
                        if (auto.simultaneousStateTest.execute())
                            active = false;
                        break;
                    case 10:

                        break;
                    default:
                        robot.stopAll();
                        break;
                }
                robot.updateSensors(active);
            } else {
                //robot.stopAll();
            }

            long loopClockTime = System.nanoTime();
            long loopTime = loopClockTime - lastLoopClockTime;
            if (loopAvg == 0)
                loopAvg = loopTime;
            else
                loopAvg = loopWeight * loopTime + (1 - loopWeight) * loopAvg;
            lastLoopClockTime = loopClockTime;

            telemetry.update();
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }

    public boolean driveStraight() {
        return robot.driveForward(true, 1, .5);
    }

    int tpmtuningstage = 0;

    public void tpmtuning() {

        switch (tpmtuningstage) {
            case 0: // todo - this probably needs work to setup the basic articulation for odometer
                    // distance tuning
                // if(robot.goToPosition(0,robot.crane.pos_reverseSafeDrive,.75,.3)){
                // }

                if (toggleAllowed(gamepad1.y, y, 1)) {
                    robot.resetMotors(true);
                }

                if (toggleAllowed(gamepad1.a, a, 1)) {
                    tpmtuningstage++;
                }
                break;
            case 1:
                if (robot.driveForward(true, 2, .35)) { // calibrate forward/backward
                    // if(robot.driveStrafe(true,2,.35)){ //calibrate strafe if capable - uncomment
                    // only one of these at a time
                    tpmtuningstage = 0;
                    robot.resetMotors(true);
                }
                break;
        }
    }

    private void initialization_initSound() {
        telemetry.addData("Please wait", "Initializing Sound");
        // telemetry.update();
        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        soundID = hardwareMap.appContext.getResources().getIdentifier("gracious", "raw",
                hardwareMap.appContext.getPackageName());
        boolean success = SoundPlayer.getInstance().preload(hardwareMap.appContext, soundID);
        if (success)
            soundState = 1;
        else
            soundState = 2;
    }

    private void demo() {
        if (gamepad1.x)
            robot.maintainHeading(gamepad1.x);
        if (gamepad1.y)
            robot.turret.maintainHeadingTurret(gamepad1.y);
    }

    int reverse = 1;

    private void joystickDrive() {
        if (notdeadzone(gamepad2.left_stick_y)) {
            robot.launcher.adjustElbowAngle(-gamepad2.left_stick_y);
        }

        if (notdeadzone(gamepad2.right_stick_y)) {
            robot.launcher.adjustBelt(-gamepad2.right_stick_y);
        }
        if (notdeadzone(gamepad2.right_stick_x)) {
            robot.turret.adjust(gamepad2.right_stick_x);
        }

        if (!joystickDriveStarted) {
            robot.resetMotors(true);
            robot.setAutonSingleStep(true);
            joystickDriveStarted = true;
            robot.launcher.setActive(true);
        }

        // robot.crane.extendToTowerHeight(0.25, Config.currentTowerHeight);

        reverse = -1;
        pwrDamper = .70;

        pwrFwd = 0;
        pwrRot = 0;

        if (notdeadzone(gamepad1.left_stick_y))
            pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        if (notdeadzone(gamepad1.right_stick_x))
            pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;

        if (nearZero(pwrFwd) && nearZero(pwrRot)) {
        } else {
            robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
        }

        if (toggleAllowed(gamepad1.a, a, 1)) {
//
        }

        if (toggleAllowed(gamepad1.x, x, 1)) {
            robot.launcher.hookToggle();
        }

        if (toggleAllowed(gamepad1.b, b, 1)) {
            robot.launcher.setElbowTargetPos(250,1);
            robot.launcher.extendToPosition(1500, 1.0);
        }

        if (toggleAllowed(gamepad1.y, y, 1) && toggleAllowed(gamepad1.dpad_down, dpad_down, 1)) {
            robot.launcher.servoGripper.setPosition(servoNormalize(800));

        }

        // gamepad2 controls

        if (toggleAllowed(gamepad2.a, a, 2)) {
            robot.launcher.toggleGripper();
        }

        if (toggleAllowed(gamepad2.b, b, 2)) {
            robot.turret.rotateCardinalTurret(true);
        }

        if (toggleAllowed(gamepad2.y, y, 2)) {
//            robot.crane.toggleSwivel();
            robot.launcher.setGripperSwivelRotation(robot.launcher.swivel_Front);
        }

        if (toggleAllowed(gamepad2.x, x, 2)) {
            robot.turret.rotateCardinalTurret(false);
        }

        if (gamepad2.left_bumper) {
            robot.launcher.swivelGripper(false);
        }

        if (gamepad2.right_bumper) {
            robot.launcher.swivelGripper(true);
        }

        if (toggleAllowed(gamepad2.dpad_up, dpad_up, 2)) {
            robot.launcher.setElbowTargetPos(2501,1);
            robot.launcher.extendToPosition(1500, 1.0);

        }

        if (toggleAllowed(gamepad2.dpad_down, dpad_down, 2)) {
            robot.launcher.setElbowTargetPos(350,1);
            robot.launcher.extendToPosition(1200, 1.0);
        }

        // turret controls
        if (notdeadzone(gamepad2.right_trigger))
            robot.turret.rotateRight(gamepad2.right_trigger * 5);
        // robot.articulate(PoseSkystone.Articulation.autoGrab);

        if (notdeadzone(gamepad2.left_trigger))
            robot.turret.rotateLeft(gamepad2.left_trigger * 5);
        // robot.articulate(PoseSkystone.Articulation.yoinkStone);

        robot.launcher.update();
        robot.turret.update(opModeIsActive());
    }

    private void joystickDrivePregameMode() {
        // robot.setAutonSingleStep(true); //single step through articulations having to
        // do with deploying

        robot.ledSystem.setColor(LEDSystem.Color.CALM);
        reverse = -1;

        pwrDamper = .70;

        // drive joysticks
        pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;

        robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);

        // turret controls - this is on gamepad2 in teleop - but on gamepad 1 for
        // prematch setup
        if (notdeadzone(gamepad1.right_trigger))
            robot.turret.rotateRight(gamepad1.right_trigger * 5);
        if (notdeadzone(gamepad1.left_trigger))
            robot.turret.rotateLeft(gamepad1.left_trigger * 5);

        // Pad1 Bumbers - Rotate Cardinal
        if (toggleAllowed(gamepad1.right_bumper, right_bumper, 1)) {
            robot.turret.rotateCardinalTurret(true);
        }
        if (toggleAllowed(gamepad1.left_bumper, left_bumper, 1)) {
            robot.turret.rotateCardinalTurret(false);

        }
        // fine adjustment of turret - this is on gamepad2 right stick in teleop - but
        // on gamepad 1 for prematch setup
        if (notdeadzone(gamepad1.left_stick_x)) {
            robot.turret.adjust(gamepad1.left_stick_x);
        }
    }

    private void logTurns(double target) {
        telemetry.addData("Error: ", target - robot.getHeading());
        // telemetry.update();
    }

    private void turnTest() {
        if (robot.rotateIMU(90, 3)) {
            telemetry.addData("Angle Error: ", 90 - robot.getHeading());
            telemetry.addData("Final Test Heading: ", robot.getHeading());
            robot.setZeroHeading();
            active = false;
        }
        telemetry.addData("Current Angle: ", robot.getHeading());
        telemetry.addData("Angle Error: ", 90 - robot.getHeading());
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
                robot.resetMotors(true);
                active = false;
            }

            if (toggleAllowed(gamepad1.right_bumper, right_bumper, 1)) {

                state++;
                if (state > 10) {
                    state = 0;
                }
                robot.resetMotors(true);
                active = false;
            }

        }

        if (toggleAllowed(gamepad1.start, startBtn, 1)) {
            robot.resetMotors(true);
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
        telemetry.addLine() .addData("autoStage", () -> auto.autoStage).addData("Game Mode", () -> GAME_MODES[gameMode]);
        telemetry.addLine() .addData("Articulation", () -> robot.getArticulation());
        telemetry.addLine().addData("elbow Current Position", () -> robot.launcher.getElbowCurrentPos());
        telemetry.addLine().addData("elbow Target Position", () -> robot.launcher.getElbowTargetPos());
        telemetry.addLine().addData("Extension Current Position", () -> robot.launcher.getExtendABobCurrentPos());
        telemetry.addLine().addData("Extension Target Position", () -> robot.launcher.getExtendABobTargetPos());
        telemetry.addLine()  .addData("chassis heading", () -> robot.getHeading());
        telemetry.addLine()  .addData("chassis ticks left", () -> robot.getLeftMotorTicks());
        telemetry.addLine()  .addData("chassis ticks right", () -> robot.getRightMotorTicks());
        telemetry.addLine()  .addData("chassis avg ticks", () -> robot.getAverageTicks());
        telemetry.addLine().addData("Loop time", "%.0fms", () -> loopAvg / 1000000);
        telemetry.addLine().addData("Turret Heading", () -> robot.turret.getHeading());
        telemetry.addLine().addData("Turret Target`s", () -> robot.turret.getTurretTargetHeading());
        telemetry.addLine().addData("Turret Current tower height: ", () -> robot.launcher.getCurrentTowerHeight());
        telemetry.addLine().addData("Turret Current angle ", () -> robot.turret.getHeading());
        telemetry.addLine() .addData("left distance ", () -> robot.getDistLeftDist());
        telemetry.addLine() .addData("right distance ", () -> robot.getDistRightDist());
        telemetry.addLine() .addData("front distance ", () -> robot.getDistForwardDist());

    }

    private void configureDashboardMatch() {
        // Configure the dashboard.

        telemetry.addLine().addData("active", () -> active).addData("state", () -> state)
                .addData("Game Mode", () -> GAME_MODES[gameMode])
                .addData("Articulation", () -> robot.getArticulation());

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

    private void ledTest() {
        int idx = (int) ((System.currentTimeMillis() / 2000) % LEDSystem.Color.values().length);
        robot.ledSystem.setColor(LEDSystem.Color.values()[idx]);
        telemetry.addData("Color", LEDSystem.Color.values()[idx].name());
    }

}
