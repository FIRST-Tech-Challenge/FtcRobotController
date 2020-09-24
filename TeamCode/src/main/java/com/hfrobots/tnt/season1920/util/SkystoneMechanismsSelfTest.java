/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

package com.hfrobots.tnt.season1920.util;

import com.acmerobotics.dashboard.config.Config;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.control.DebouncedButton;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.control.OnOffButton;
import com.hfrobots.tnt.corelib.control.RangeInput;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.season1920.CapstoneMechanism;
import com.hfrobots.tnt.season1920.DeliveryMechanism;
import com.hfrobots.tnt.season1920.FoundationGripMechanism;
import com.hfrobots.tnt.season1920.ParkingSticks;
import com.hfrobots.tnt.season1920.SkystoneGrabber;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * An op-mode that runs the robot mechanisms (open and closed loops) to
 * make sure they are working correctly
 */
@Config
@TeleOp(name="Mech Selftest", group = "util")
@SuppressWarnings("unused")
public class SkystoneMechanismsSelfTest extends LinearOpMode {

    private DeliveryMechanism deliveryMechanism;

    private FoundationGripMechanism foundationGripMechanism;

    private SkystoneGrabber skystoneGrabber;

    private ParkingSticks parkingSticks;

    private CapstoneMechanism capstoneMechanism;

    @Override
    public void runOpMode() throws InterruptedException {
        Ticker ticker = createAndroidTicker();

        NinjaGamePad gamePad = new NinjaGamePad(gamepad1);

        DebouncedButton nextStepButton = new DebouncedButton(gamePad.getAButton());
        AutoOnOffButton armInPostion = new AutoOnOffButton();
        AutoOnOffButton armOutPostion = new AutoOnOffButton();
        AutoOnOffButton grip = new AutoOnOffButton();
        AutoOnOffButton ungrip = new AutoOnOffButton();
        AutoOnOffButton stow = new AutoOnOffButton();
        AutoLiftThrottle liftThrottle = new AutoLiftThrottle();

        AutoOnOffButton dummyButton = new AutoOnOffButton();

        AutoOnOffButton unsafeButton = new AutoOnOffButton();
        AutoOnOffButton capstoneButton = new AutoOnOffButton();

        RealSimplerHardwareMap simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        deliveryMechanism = new DeliveryMechanism(simplerHardwareMap, telemetry, ticker);
        this.deliveryMechanism.setArmInPostion(new DebouncedButton(armInPostion));
        this.deliveryMechanism.setArmOutPostion(new DebouncedButton(armOutPostion));
        this.deliveryMechanism.setGrip(new DebouncedButton(grip));
        this.deliveryMechanism.setUngrip(new DebouncedButton(ungrip));
        this.deliveryMechanism.setLiftThrottle(liftThrottle);
        this.deliveryMechanism.setStow(new DebouncedButton(stow));
        this.deliveryMechanism.setRotateWrist(new DebouncedButton(dummyButton));
        this.deliveryMechanism.setUnsafe(dummyButton);

        this.foundationGripMechanism = new FoundationGripMechanism(simplerHardwareMap);
        this.skystoneGrabber = new SkystoneGrabber(simplerHardwareMap);
        this.parkingSticks = new ParkingSticks(simplerHardwareMap);

        this.capstoneMechanism = new CapstoneMechanism(simplerHardwareMap, telemetry, ticker);
        this.capstoneMechanism.setUnsafeButton(unsafeButton);
        this.capstoneMechanism.setDropButton(new DebouncedButton(capstoneButton));

        telemetry.log().add("Press play to begin the mechanism selftest");
        telemetry.log().add("Make sure the area around the mechanisms is clear");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        telemetry.log().clear();
        telemetry.log().add("Running...");
        telemetry.update();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to grab skystone");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        skystoneGrabber.grab();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to stow skystone grabber");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        skystoneGrabber.stow();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to grip");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        doGrip(grip);

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to lift");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        liftToMax(liftThrottle);

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to move arm out");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        doArmOut(armOutPostion);

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to stow");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        doStow(stow);

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to deploy foundation gripper");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        foundationGripMechanism.down();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to stow foundation gripper");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        foundationGripMechanism.up();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to init foundation gripper");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        foundationGripMechanism.initPos();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to drop capstone");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        capstoneMechanism.periodicTask();
        unsafeButton.buttonOn = true;
        capstoneButton.buttonOn = true;
        capstoneMechanism.periodicTask();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to hold capstone");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        capstoneButton.buttonOn = false;
        capstoneMechanism.periodicTask();
        unsafeButton.buttonOn = true;
        capstoneButton.buttonOn = true;
        capstoneMechanism.periodicTask();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to stow parking");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        parkingSticks.stow();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to deploy parking");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        parkingSticks.deploy();

        telemetry.log().clear();
        telemetry.log().add("Press (a) button to stow parking");
        telemetry.update();

        while (!isStopRequested() && !nextStepButton.getRise());

        parkingSticks.stow();

        telemetry.log().clear();
        telemetry.log().add("Tests complete");
        telemetry.update();

        while (!isStopRequested()) {
            idle();
        }
    }

    private void doArmOut(AutoOnOffButton armOutPostion) {
        deliveryMechanism.periodicTask();
        armOutPostion.buttonOn = true;
        for (int i = 0; i < 3; i++) {
            deliveryMechanism.periodicTask();
        }
        armOutPostion.buttonOn = false;
        deliveryMechanism.periodicTask();
    }

    private void liftToMax(AutoLiftThrottle liftThrottle) {
        liftThrottle.position = -.5F;

        while (!isStopRequested()) {
            deliveryMechanism.periodicTask();
            String stateName = deliveryMechanism.getCurrentStateName();

            if ("AtMaxState".equals(stateName)) {
                break;
            }

            telemetry.log().clear();
            telemetry.log().add(stateName);
            telemetry.update();
        }
    }

    private void doStow(AutoOnOffButton stow) {
        stow.buttonOn = true;

        while (!isStopRequested()) {
            deliveryMechanism.periodicTask();
            String stateName = deliveryMechanism.getCurrentStateName();

            if ("LoadingState".equals(stateName)) {
                break;
            }

            telemetry.log().clear();
            telemetry.log().add(stateName);
            telemetry.update();
        }

        stow.buttonOn = false;
    }


    private void doGrip(AutoOnOffButton grip) {
        deliveryMechanism.periodicTask();
        grip.buttonOn = true;
        for (int i  = 0; i < 3; i++) {
            deliveryMechanism.periodicTask();
        }
        grip.buttonOn = false;
        deliveryMechanism.periodicTask();
    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    class AutoOnOffButton implements OnOffButton {
        boolean buttonOn = false;

        @Override
        public boolean isPressed() {
            return buttonOn;
        }
    }

    class AutoLiftThrottle implements RangeInput {
        float position;

        @Override
        public float getPosition() {
            return position;
        }

        @Override
        public float getMaxPosition() {
            return 1;
        }

        @Override
        public float getMinPosition() {
            return -1;
        }
    }
}
