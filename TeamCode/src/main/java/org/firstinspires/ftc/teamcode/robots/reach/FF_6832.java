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
package org.firstinspires.ftc.teamcode.robots.reach;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robots.reach.utils.Constants;
import org.firstinspires.ftc.teamcode.util.CsvLogKeeper;
import org.firstinspires.ftc.teamcode.util.utilMethods;

import static org.firstinspires.ftc.teamcode.robots.reach.utils.Constants.*;
import static org.firstinspires.ftc.teamcode.util.utilMethods.*;

/**
 * This file contains the code for Iron Reign's main OpMode, used for both
 * TeleOp and Autonomous.
 */
@Disabled
@TeleOp(name = "_Reach_6832", group = "Challenge")

public class FF_6832 extends OpMode {
    //Important Variables
    private ElapsedTime runtime = new ElapsedTime();
    private ReachPose.RobotType currentBot = ReachPose.RobotType.Reach;
    private ReachPose robot;
    private Autonomous auto;
    public CsvLogKeeper logger;

    //global state variables:
    private boolean active = true;
    private boolean joystickDriveStarted = false;
    static public int state = 0;

    // loop time profile
    long lastLoopClockTime;
    double loopAvg = 0;
    private static final double loopWeight = .1;

    // drive train control variables
    private double pwrFwd = 0;
    private double pwrRot = 0;

    // values associated with the buttons in the toggleAllowed method
    private boolean[] buttonSavedStates = new boolean[20];
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
    private int dpad_left_2 = 16;
    private int dpad_right_2 = 17;
    private int dpad_up_2 = 18;
    private int dpad_down_2 = 19;
    private int y_2 = 20;
    private int b_2 = 21;
    private int a_2 = 22;
    private int x_2 = 23;

    enum gamepadButtons{
        a,
        b,
        x,
        y,
        dpad_down,
        dpad_up,
        dpad_left,
        dpad_right,
        left_bumper,
        right_bumper,
        startBtn,
        left_trigger,
        right_trigger,
        back_button,
        left_stick_button,
        right_stick_button;
    }

    // values associated with the buttons in the toggleAllowedGP2 method
    private boolean[] buttonSavedStates2 = new boolean[24];


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
        //important instantiation
        robot = new ReachPose(currentBot);
        auto = new Autonomous(robot, dummyT);
        logger = new CsvLogKeeper("test",3,"tps, armTicks, targetDistance");

        //initializaion method calls
        robot.init(this.hardwareMap, telemetry);
        robot.resetMotors(true);
        auto.visionProviderFinalized = false;
        robot.setupDriverTelemetry(active, state, ALLIANCE, loopAvg);
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        stateSwitch();

        if (active) {
            joystickDrivePregameMode();
        }

        telemetry.update();

        robot.update();
        robot.sendTelemetry();
    }

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

        lastLoopClockTime = System.nanoTime();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //orange
            stateSwitch();
            if (active) {
                switch (state) {
                    case 0: // auton full
                        joystickDrive();
                        break;
                    case 1: // teleop
                        if (auto.testAuto.execute()) {
                            active = false;
                            state = 0;
                        }
                        break;
                    case 2:
                        break;
                    case 3:
                        break;
                    case 4:
                        break;
                    case 5:
                        break;
                    case 6:
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
                robot.update();
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

    // the method that controls the main state of the robot; must be called in the
    // main loop outside of the main switch
    private void stateSwitch() {
        if (!active) {
            if (toggleAllowed(gamepad1.left_bumper, left_bumper, 1))
                state = utilMethods.boundInt(state-1,0, Constants.numGameStates);
            if (toggleAllowed(gamepad1.right_bumper, right_bumper, 1))
                state = utilMethods.boundInt(state+1,0, Constants.numGameStates);
        }

        if (toggleAllowed(gamepad1.start, startBtn, 1)) {
            active = !active;
        }
    }

    private void joystickDrive() { //apple
        //initialize certain things
        if (!joystickDriveStarted) {
            joystickDriveStarted = true;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_RAINBOW_PALETTE);
        }

        //reset the power values for this loop
        pwrFwd = 0;
        pwrRot = 0;


        //pick up joystick inputs and send values to mixer

        if (notdeadzone(gamepad1.left_stick_y))
            pwrFwd = gamepad1.left_stick_y;
        if (notdeadzone(gamepad1.right_stick_x))
            pwrRot = gamepad1.right_stick_x;

        if (nearZero(pwrFwd) && nearZero(pwrRot)) {
            robot.driveMixerTrike2(0,0);
        } else {
            robot.driveMixerTrike2(pwrFwd,pwrRot);
        }


        //button inputs

        if(toggleAllowed(gamepad1.x,x,1)) {
            //robot.toggleDuckSpinner(); todo-add this back in properly
        }
    }

    private void joystickDrivePregameMode() {
//        robot.driveMixerTrike(pwrFwd, pwrRot);

        //this is all code to set positions
        if(toggleAllowed(gamepad1.x,x,1)) {
            ALLIANCE = Constants.Alliance.BLUE;
            ALLIANCE_INT_MOD=-1;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        }

        //press red button to set red alliance
        if(toggleAllowed(gamepad1.b,b,1)) {
            ALLIANCE = Constants.Alliance.RED;
            ALLIANCE_INT_MOD=1;
            robot.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
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
}
