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

package com.hfrobots.tnt.season1920;

import android.content.Context;
import android.util.Log;

import com.google.common.base.Ticker;
import com.google.common.collect.ImmutableSet;
import com.hfrobots.tnt.corelib.chaosninja.ChaosConfigSaver;
import com.hfrobots.tnt.corelib.chaosninja.ChaosController;
import com.hfrobots.tnt.corelib.control.ChaosNinjaLandingState;
import com.hfrobots.tnt.corelib.control.KonamiCode;
import com.hfrobots.tnt.corelib.control.NinjaGamePad;
import com.hfrobots.tnt.corelib.drive.mecanum.RoadRunnerMecanumDriveREV;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.hfrobots.tnt.corelib.util.RealSimplerHardwareMap;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Set;

import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

@TeleOp(name="00 Skystone Teleop")
@SuppressWarnings("unused")
public class SkystoneTeleop extends OpMode {
    private OpenLoopMecanumKinematics kinematics;

    private RoadRunnerMecanumDriveREV driveBase;

    private DriverControls driverControls;

    protected OperatorControls operatorControls;

    private DeliveryMechanism deliveryMechanism;

    private FoundationGripMechanism foundationGripMechanism;

    private StationKeeping stationKeeping;

    protected StatsDMetricSampler metricSampler;

    private Ticker ticker;

    private KonamiCode konamiCode;

    private ChaosNinjaLandingState chaosNinja;
    private ChaosController chaosController;

    private NinjaGamePad driversGamepad;
    private NinjaGamePad operatorsGamepad;

    private SimplerHardwareMap simplerHardwareMap;

    protected SkystoneGrabber skystoneGrabber;

    private CapstoneMechanism capstoneMechanism;

    private ParkingSticks parkingSticks;

    @Override
    public void init() {
        ticker = createAndroidTicker();

        simplerHardwareMap = new RealSimplerHardwareMap(this.hardwareMap);
        driveBase = new RoadRunnerMecanumDriveREV(new SkystoneDriveConstants(), simplerHardwareMap, false);
        kinematics = new OpenLoopMecanumKinematics(driveBase);

        stationKeeping = new StationKeeping(simplerHardwareMap, telemetry);

        foundationGripMechanism = new FoundationGripMechanism(simplerHardwareMap);

        driversGamepad = new NinjaGamePad(gamepad1);

        parkingSticks = new ParkingSticks(simplerHardwareMap);

        driverControls = DriverControls.builder().driversGamepad(driversGamepad)
                .kinematics(kinematics)
                .foundationGripMechanism(foundationGripMechanism)
                .stationKeeping(stationKeeping)
                .parkingSticks(parkingSticks)
                .build();

        deliveryMechanism = new DeliveryMechanism(simplerHardwareMap, telemetry, ticker);

        capstoneMechanism = new CapstoneMechanism(simplerHardwareMap, telemetry, ticker);

        operatorsGamepad = new NinjaGamePad(gamepad2);

        operatorControls = OperatorControls.builder().operatorsGamepad(operatorsGamepad)
                .deliveryMechanism(deliveryMechanism)
                .capstoneMechanism(capstoneMechanism)
                .build();

        chaosNinja = new ChaosNinjaLandingState(driversGamepad, telemetry);
        konamiCode = new KonamiCode(driversGamepad, chaosNinja, ticker, telemetry);

        skystoneGrabber = new SkystoneGrabber(simplerHardwareMap);
    }

    @Override
    public void init_loop() {
        super.init_loop();

        konamiCode.periodicTask();

        telemetry.update();
    }

    private Ticker createAndroidTicker() {
        return new Ticker() {
            public long read() {
                return android.os.SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    @Override
    public void start() {
        super.start();

        deliveryMechanism.ungripblock(); // in init, it's gripped.

        try {
            handleChaos();
        } catch (Throwable t) {
            // FIRST, do no harm
            Log.e(LOG_TAG, "Chaos config failure", t);
            chaosController = null;
        }
    }

    private void configureChaosNinja(Set<String> drivebaseMotors,
                                     Set<Set<String>> mechanismMotors,
                                     Set<String> servos) {
        // TODO/FIXME Get more of this out of the op-mode to make it more reusable
        Context appContext = hardwareMap.appContext;

        ChaosConfigSaver.Config chaosConfig = new ChaosConfigSaver(appContext)
                .getSavedOrNewConfiguration(chaosNinja);

        if (chaosConfig.applyConfiguration) {
            if (chaosConfig.metricsActivated) {
                setupMetricsSampler();

            } else {
                Log.i(LOG_TAG, "No metrics requested, not enabling");

                metricSampler = null;
            }

            if (chaosConfig.challengeLevel > 0) {
                chaosController = new ChaosController(
                        chaosConfig,
                        drivebaseMotors,
                        mechanismMotors,
                        servos,
                        ticker,
                        simplerHardwareMap, telemetry);
            } else {
                chaosController = null; // normal mode
            }
        }
    }

    private void handleChaos() {
        if (true) {
            Set<String> intakeMotors = ImmutableSet.of("leftIntakeMotor", "rightIntakeMotor");

            Set<String> liftMotor = ImmutableSet.of("liftMotor");

            Set<String> servos = ImmutableSet.of("fingerServo",
                    "ejectorServo", "parkingStick0", "parkingStick1",
                    "capstoneServo");

            configureChaosNinja(ImmutableSet.of(
                    "leftFrontDriveMotor",
                    "rightFrontDriveMotor",
                    "leftRearDriveMotor",
                    "rightRearDriveMotor"),
                    ImmutableSet.of(intakeMotors, liftMotor),
                    servos);
        }
    }

    protected void setupMetricsSampler() {
        Log.i(LOG_TAG, "Metrics requested, enabling");
        metricSampler = new StatsDMetricSampler(hardwareMap, driversGamepad,
                operatorsGamepad);
    }

    @Override
    public void stop() {
        super.stop();
    }

    @Override
    public void loop() {
        if (chaosController != null) {
            chaosController.periodicTask();
        }

        skystoneGrabber.stow(); // keep the grabber stowed at all times in tele-op

        driverControls.periodicTask();
        operatorControls.periodicTask();

        if (metricSampler != null) {
            metricSampler.doSamples();
        }

        updateTelemetry(telemetry);
    }
}
