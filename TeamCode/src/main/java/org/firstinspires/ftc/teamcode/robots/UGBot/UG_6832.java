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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robots.UGBot.utils.Constants;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.OpenCVIntegration;
import org.firstinspires.ftc.teamcode.robots.UGBot.vision.StackHeight;
import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;

import static org.firstinspires.ftc.teamcode.util.Conversions.nearZero;
import static org.firstinspires.ftc.teamcode.util.Conversions.notdeadzone;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both
 * TeleOp and Autonomous.
 */

@TeleOp(name = "AAAUltimateGoal_6832", group = "Challenge") // @Autonomous(...) is the other common choice
// @Autonomous
@Config
public class UG_6832 extends OpMode {

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

    public CsvLogKeeper logger;

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

    private FtcDashboard dashboard;

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
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initializing " + currentBot + "...");
        telemetry.addData("Status", "Hold right_trigger to enable debug mode");
        telemetry.update();

        robot = new PoseUG(currentBot);
        robot.init(this.hardwareMap);

        auto = new Autonomous(robot, dummyT, gamepad1);

        logger = new CsvLogKeeper("test",3,"tps, armTicks, targetDistance");


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

        dashboard = FtcDashboard.getInstance();
        robot.resetMotors(true);
        auto.visionProviderFinalized = false;
    }
        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {


            stateSwitch();

            if (active) {

                // we can do very basic driving to get to calibration position
                // turret and drive controls on gamepad1 only since we don't always have 2 pads
                // for auton testing

                // this test suppresses pregame driving while a calibration articulation is
                // active
                if (robot.articulation == PoseUG.Articulation.manual)
                    joystickDrivePregameMode();

                if(!auto.visionProviderFinalized) {
                    auto.visionProviderState = 0;
                    auto.initVisionProvider();
                } else {
                    auto.sample();
                    telemetry.addData("Vision", "Prep detection: %s%s", auto.height,
                            auto.height == StackHeight.NONE_FOUND ? " (NONE_FOUND)" : "");
                    robot.setDetection(auto.height);
                    if(auto.vp instanceof OpenCVIntegration) {
                        OpenCVIntegration vp = (OpenCVIntegration) auto.vp;
                        robot.setVisionTimes(new long[] {
                                vp.pipeline.normalizeTime - vp.pipeline.cropTime,
                                vp.pipeline.blurTime - vp.pipeline.normalizeTime,
                                vp.pipeline.hsvTime - vp.pipeline.blurTime,
                                vp.pipeline.contourTime - vp.pipeline.hsvTime,
                                vp.pipeline.momentsTime - vp.pipeline.contourTime
                        });
                        telemetry.addData("Frame Count", vp.camera.getFrameCount());
                        telemetry.addData("FPS", String.format("%.2f", vp.camera.getFps()));
                        telemetry.addData("Total frame time ms", vp.camera.getTotalFrameTimeMs());
                        telemetry.addData("Pipeline time ms", vp.camera.getPipelineTimeMs());
                        telemetry.addData("Overhead time ms", vp.camera.getOverheadTimeMs());
                        telemetry.addData("Theoretical max FPS", vp.camera.getCurrentPipelineMaxFps());

                        robot.setFrameCount(vp.camera.getFrameCount());
                        robot.setVisionFPS(vp.camera.getFps());
                        robot.setTotalFrameTimeMs(vp.camera.getTotalFrameTimeMs());
                        robot.setPipelineTimeMs(vp.camera.getPipelineTimeMs());
                        robot.setOverheadTimeMs(vp.camera.getOverheadTimeMs());
                        robot.setCurrentPipelineMaxFps(vp.camera.getCurrentPipelineMaxFps());

                    }
                }

//                if (auto.visionProviderFinalized) {
//                    StackHeight sp = auto.vp.detect();
//                    if (sp != StackHeight.NONE_FOUND)
//                        initStackHeightTest = sp;
//                    telemetry.addData("Vision", "Prep detection: %s%s", initStackHeightTest,
//                            sp == StackHeight.NONE_FOUND ? " (NONE_FOUND)" : "");
//                    robot.setDetection(initStackHeightTest);
//
//                } else {
//                    auto.visionProviderState = 0;
//                    auto.initVisionProvider(); // this is blocking
//                }

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
            robot.sendTelemetry();


        } // end of stuff that happens during Init, but before Start

        //
        // THIS SECTION EXECUTES ONCE RIGHT AFTER START IS PRESSED
        //
        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            runtime.reset();


            if (auto.vp == null) {
                auto.initDummyVisionProvider(); // this is blocking
            }

            auto.vp.reset();



            robot.launcher.restart(.4, .5);

            lastLoopClockTime = System.nanoTime();
        }

        //
        // END OF SECTION THAT EXECUTES ONCE RIGHT AFTER START IS PRESSED
        //
    int autoState = 0;
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {


            stateSwitch();
            if (active) {
                switch (state) {
                    case 0: // auton full
                        joystickDrive();
                        break;
                    case 1: // teleop
                        if (auto.AutoFull.execute()) {
                            active = false;
                            state = 0;
                        }
                        break;
                    case 2:
                        if (auto.AutoTest.execute()) {
                            active = false;
                            state = 0;
                        }
                        break;
                    case 4:
                        testRamp();
                        break;
                    case 6:
                        demo();
                        break;
                    case 7:
                        break;
                    case 8:
                        break;
                    case 9:
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

        }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
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
            robot.turret.maintainHeadingTurret();
    }

    int reverse = 1;
    String message = "";

    int lastCachedArmTiccs = 0;
    double lastCachedTPS = 0;
    boolean cacheValidated = false;

    boolean rotate = false;
    private void testRamp() {
        reverse = -1;
        pwrDamper = .70;

        pwrFwd = 0;
        pwrRot = 0;

        if (notdeadzone(gamepad1.left_stick_y))
            pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        if (notdeadzone(gamepad1.right_stick_x))
            pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;

        if (nearZero(pwrFwd) && nearZero(pwrRot)) {
            robot.driveMixerDiffSteer(0, 0);
        } else {
            robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
        }

        if (gamepad1.right_trigger > .01)
            robot.turret.rotateRight(gamepad1.right_trigger * 2);

        if (gamepad1.left_trigger > .01)
            robot.turret.rotateLeft(gamepad1.left_trigger * 2);

        if (notdeadzone(gamepad1.right_stick_y)) {
            robot.launcher.adjustElbowAngle(-gamepad1.right_stick_y);
        }

        if(toggleAllowed(gamepad1.b, b, 1))
            rotate = !rotate;

        if(toggleAllowed(gamepad1.dpad_right,dpad_right,1))
            robot.setTarget(Constants.Target.NONE);
        if(toggleAllowed(gamepad1.dpad_up,dpad_up,1))
            robot.setTarget(Constants.Target.HIGH_GOAL);
        if(toggleAllowed(gamepad1.dpad_left,dpad_left,1)){
            switch(robot.getTarget()){
                case FIRST_POWER_SHOT:
                    robot.setTarget(Constants.Target.SECOND_POWER_SHOT);
                    break;
                case SECOND_POWER_SHOT:
                    robot.setTarget(Constants.Target.THIRD_POWER_SHOT);
                    break;
                case THIRD_POWER_SHOT:
                    robot.setTarget(Constants.Target.FIRST_POWER_SHOT);
                    break;
                default:
                    robot.setTarget(Constants.Target.FIRST_POWER_SHOT);
            }
        }
        if(toggleAllowed(gamepad1.dpad_down,dpad_down,1)) {
            switch(robot.getTarget()){
                case MID_GOAL:
                    robot.setTarget(Constants.Target.LOW_GOAL);
                    break;
                case LOW_GOAL:
                    robot.setTarget(Constants.Target.MID_GOAL);
                    break;
                default:
                    robot.setTarget(Constants.Target.MID_GOAL);
            }
        }

        if(rotate) {
            robot.turret.rotateRight(2);
            if(robot.autoIntakeState == 0)
                robot.articulate(PoseUG.Articulation.autoIntake);
        }

//        robot.launcher.update();
//        robot.turret.update(); //todo- make sure there wasn't a reason this was here
//        robot.intake.update();
    }



    private void joystickDrive() { //apple

        if (!joystickDriveStarted) {
            robot.setAutonSingleStep(true);
            joystickDriveStarted = true;
            robot.launcher.setActive(true);
            robot.articulate(PoseUG.Articulation.makeIntakeOuttake);
        }


        reverse = -1;
        pwrDamper = .70;

        pwrFwd = 0;
        pwrRot = 0;

        if(toggleAllowed(gamepad1.left_bumper, left_bumper, 1)){
            if(robot.launcher.gripperTargetPos == Constants.WOBBLE_GRIPPER_OPEN)
                robot.launcher.wobbleRelease();
            else
                robot.launcher.wobbleGrip();
        }

        if(toggleAllowed(gamepad1.right_bumper, right_bumper, 1)){
            if(Constants.GRIPPER_IS_OUT)
                robot.launcher.setGripperOutTargetPos(Constants.GRIPPER_IN_POS);
            else
                robot.launcher.setGripperOutTargetPos(Constants.GRIPPER_OUT_POS);
        }

        if (notdeadzone(gamepad1.left_stick_y)) {
            pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
        }

        if(robot.isTented){
            if(gamepad1.left_stick_y <= 0)
                robot.intake.setIntakeSpeed(gamepad1.left_stick_y * Constants.__ATMEP2);
            if(gamepad1.left_stick_y >= 0)
                robot.intake.setIntakeSpeed(gamepad1.left_stick_y * Constants.__ATMEP);
        }


        if (notdeadzone(gamepad1.right_stick_x))
            pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;


            if (nearZero(pwrFwd) && nearZero(pwrRot)) {
                robot.driveMixerDiffSteer(0, 0);
            } else {
                robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
                }

        //region good logging example

//        if(toggleAllowed(gamepad1.a, a, 1) && gamepad1.y){
//            robot.articulate(PoseUG.Articulation.toggleTrigger);
//            lastCachedTPS = robot.launcher.getFlywheelTPS();
//            lastCachedArmTiccs= robot.launcher.getElbowCurrentPos();
//        }
//
//        if(toggleAllowed(gamepad2.a, a, 2)){
//            cacheValidated = true;
//        }

//        if(cacheValidated = true){
//            logger.UpdateLog(Double.toString(lastCachedTPS) + ","  + Double.toString(lastCachedArmTiccs) + "," + "idkguess");
//            cacheValidated = false;
//        }
        //endregion
        if(toggleAllowed(gamepad1.b, b, 1))
            robot.articulate(PoseUG.Articulation.toggleTrigger);
        if(toggleAllowed(gamepad1.a, a, 1))
            robot.flywheelIsActive = !robot.flywheelIsActive;
        if(toggleAllowed(gamepad1.y, y, 1)) {
            robot.articulate(PoseUG.Articulation.autoIntake);
        }
        if(toggleAllowed(gamepad1.x,x,1)){
            robot.articulate(PoseUG.Articulation.setUpTent);
        }

        if (notdeadzone(gamepad1.right_stick_y)) {
            robot.launcher.adjustElbowAngle(-gamepad1.right_stick_y);
        }



        if (gamepad1.right_trigger > .01)
            robot.turret.rotateRight(gamepad1.right_trigger * 2);

        if (gamepad1.left_trigger > .01)
            robot.turret.rotateLeft(gamepad1.left_trigger * 2);

        if(toggleAllowed(gamepad1.dpad_right,dpad_right,1))
            robot.setTarget(Constants.Target.NONE);
        if(toggleAllowed(gamepad1.dpad_up,dpad_up,1))
            robot.setTarget(Constants.Target.HIGH_GOAL);
        if(toggleAllowed(gamepad1.dpad_left,dpad_left,1)){
            switch(robot.getTarget()){
                case FIRST_POWER_SHOT:
                    robot.setTarget(Constants.Target.SECOND_POWER_SHOT);
                    break;
                case SECOND_POWER_SHOT:
                    robot.setTarget(Constants.Target.THIRD_POWER_SHOT);
                    break;
                case THIRD_POWER_SHOT:
                    robot.setTarget(Constants.Target.FIRST_POWER_SHOT);
                    break;
                default:
                    robot.setTarget(Constants.Target.FIRST_POWER_SHOT);
            }
        }
        if(toggleAllowed(gamepad1.dpad_down,dpad_down,1)) {
            switch(robot.getTarget()){
                case MID_GOAL:
                    robot.setTarget(Constants.Target.LOW_GOAL);
                    break;
                case LOW_GOAL:
                    robot.setTarget(Constants.Target.MID_GOAL);
                    break;
                default:
                    robot.setTarget(Constants.Target.MID_GOAL);
            }
        }

//        robot.launcher.update();
//        robot.turret.update(); //todo- make sure there wasn't a reason this was here
//        robot.intake.update();
    }

    private void gripperJoystickDrive() {
        pwrDamper = .70;

        pwrFwd = 0;
        pwrRot = 0;

        if (notdeadzone(gamepad1.left_stick_y)) {
            pwrFwd = reverse * direction * pwrDamper * gamepad1.left_stick_y;
            robot.articulate(PoseUG.Articulation.manual);
        }

        if (notdeadzone(gamepad1.right_stick_x)) {
            pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;
            robot.articulate(PoseUG.Articulation.manual);
        }

        if (nearZero(pwrFwd) && nearZero(pwrRot) && robot.getArticulation() != PoseUG.Articulation.manual) {
            robot.driveMixerDiffSteer(0, 0);
        } else {
            robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
        }


        if(toggleAllowed(gamepad1.left_bumper, left_bumper, 1)){
            if(robot.launcher.gripperTargetPos == Constants.WOBBLE_GRIPPER_OPEN)
                robot.launcher.wobbleRelease();
            else
                robot.launcher.wobbleGrip();
        }

        if (notdeadzone(gamepad1.right_stick_y)) {
            robot.launcher.adjustElbowAngle(-gamepad1.right_stick_y);
        }

        if(toggleAllowed(gamepad1.right_bumper, right_bumper, 1))
            robot.launcher.setGripperOutTargetPos(Constants.GRIPPER_IN_POS); //todo-write the articulations

        if(toggleAllowed(gamepad1.b,b,1))
            robot.launcher.setGripperOutTargetPos(Constants.GRIPPER_IN_POS); //todo-write the articulations
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

        if (notdeadzone(gamepad1.right_stick_y)) {
            robot.launcher.adjustElbowAngle(-gamepad1.right_stick_y);
        }
        // fine adjustment of turret - this is on gamepad2 right stick in teleop - but
        // on gamepad 1 for prematch setup
        if (notdeadzone(gamepad1.left_stick_x)) {
            robot.turret.adjust(gamepad1.left_stick_x);
        }

        if(toggleAllowed(gamepad1.x,x,1))
            robot.alignmentRun(.5,-0.4572 * Constants.WALL_FOLLOW_MULTIPLIER, -robot.getDistRightDist() * Constants.WALL_FOLLOW_MULTIPLIER,true, 2.7432);
        if(toggleAllowed(gamepad1.a,a,1)){
            robot.articulate(PoseUG.Articulation.makeIntakeOuttake);
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
        telemetry.addLine() .addData("autoStage", () -> auto.autoStage).addData("Game Mode", () -> GAME_MODES[gameMode]);
        telemetry.addLine() .addData("Articulation", () -> robot.getArticulation());
        telemetry.addLine().addData("elbow Current Position", () -> robot.launcher.getElbowCurrentPos());
        telemetry.addLine().addData("elbow Target Position", () -> robot.launcher.getElbowTargetPos());
        telemetry.addLine()  .addData("chassis heading", () -> robot.getHeading());
        telemetry.addLine()  .addData("chassis ticks left", () -> robot.getLeftMotorTicks());
        telemetry.addLine()  .addData("chassis ticks right", () -> robot.getRightMotorTicks());
        telemetry.addLine()  .addData("chassis avg ticks", () -> robot.getAverageTicks());
        telemetry.addLine().addData("Loop time", "%.0fms", () -> loopAvg / 1000000);
        telemetry.addLine().addData("Turret Heading", () -> robot.turret.getHeading());
        telemetry.addLine().addData("Turret Target`s", () -> robot.turret.getTurretTargetHeading());
        telemetry.addLine().addData("Turret Current angle ", () -> robot.turret.getHeading());
        telemetry.addLine().addData("Current Target", () -> robot.getTarget());


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
