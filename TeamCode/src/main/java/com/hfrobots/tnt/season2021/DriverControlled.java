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

package com.hfrobots.tnt.season2021;

import android.util.Log;

import com.ftc9929.corelib.control.NinjaGamePad;
import com.ftc9929.metrics.RobotMetricsSampler;
import com.ftc9929.metrics.StatsdMetricsReporter;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.metrics.StatsDMetricSampler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

import static com.ftc9929.corelib.Constants.LOG_TAG;

@TeleOp(name = "00 UltGoal TeleOp")
public class DriverControlled extends OpMode {

    private Drivebase drivebase;

    private DriverControls driverControls;

    private OperatorControls operatorControls;

    private RevBlinkinLedDriver blinkinLed;

    private StatsDMetricSampler legacyMetricsSampler;

    private RobotMetricsSampler newMetricsSampler;

    private boolean useLegacyMetricsSampler = true;

    private List<LynxModule> allHubs;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);

        NinjaGamePad driversGamepad = new NinjaGamePad(gamepad1);

        driverControls = DriverControls.builder()
                .driversGamepad(driversGamepad)
                .kinematics(drivebase).build();

        ScoringMechanism scoringMechanism = ScoringMechanism.builder()
                .hardwareMap(hardwareMap)
                .telemetry(telemetry)
                .ticker(Ticker.systemTicker()).build();

        scoringMechanism.toDeployedPosition();

        WobbleGoal wobbleGoal = WobbleGoal.builder().hardwareMap(hardwareMap)
                .telemetry(telemetry).ticker(Ticker.systemTicker()).build();

        NinjaGamePad operatorGamepad = new NinjaGamePad(gamepad2);

        operatorControls = OperatorControls.builder().operatorGamepad(operatorGamepad)
                .scoringMechanism(scoringMechanism)
                .wobbleGoal(wobbleGoal).build();

        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);

        try {
            if (useLegacyMetricsSampler) {
                legacyMetricsSampler = new StatsDMetricSampler(hardwareMap, driversGamepad, operatorGamepad);
            } else {
                StatsdMetricsReporter metricsReporter = StatsdMetricsReporter.builder()
                        .metricsServerHost("192.168.43.78").
                                metricsServerPortNumber(8126).build();

                newMetricsSampler = RobotMetricsSampler.builder()
                        .metricsReporter(metricsReporter)
                        .hardwareMap(hardwareMap)
                        .driverControls(driversGamepad)
                        .operatorControls(operatorGamepad).build();

            }
        } catch (Exception ex) {
            Log.w(LOG_TAG, "Unable to setup metrics sampler", ex);
        }

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            Log.d(LOG_TAG, String.format("Setting hub %s to BulkCachingMode.MANUAL", hub));
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    @Override
    public void init_loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

        // Let operator work with wobble goal gripper during setup
        operatorControls.periodicTask();
    }

    private void clearHubsBulkCaches() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    @Override
    public void loop() {
        clearHubsBulkCaches(); // important, do not remove this line, or reads from robot break!

        driverControls.periodicTask();
        operatorControls.periodicTask();

        if (useLegacyMetricsSampler) {
            if (legacyMetricsSampler != null) {
                legacyMetricsSampler.doSamples();
            }
        } else {
            if (newMetricsSampler != null) {
                newMetricsSampler.doSamples();
            }
        }

        telemetry.update();
    }
}
