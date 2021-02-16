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

import com.ftc9929.corelib.control.NinjaGamePad;
import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.sensors.TotalCurrentSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp(name = "00 UltGoal TeleOp")
public class DriverControlled extends OpMode {

    private Drivebase drivebase;

    private DriverControls driverControls;

    private OperatorControls operatorControls;

    private RevBlinkinLedDriver blinkinLed;

    private TotalCurrentSensor totalCurrentSensor;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap);

        totalCurrentSensor = new TotalCurrentSensor(hardwareMap);

        driverControls = DriverControls.builder()
                .driversGamepad(new NinjaGamePad(gamepad1))
                .kinematics(drivebase).build();

        ScoringMechanism scoringMechanism = ScoringMechanism.builder()
                .hardwareMap(hardwareMap)
                .telemetry(telemetry)
                .ticker(Ticker.systemTicker()).build();

        WobbleGoal wobbleGoal = WobbleGoal.builder().hardwareMap(hardwareMap)
                .telemetry(telemetry).ticker(Ticker.systemTicker()).build();

        operatorControls = OperatorControls.builder().operatorGamepad(new NinjaGamePad(gamepad2))
                .scoringMechanism(scoringMechanism)
                .wobbleGoal(wobbleGoal).build();

        blinkinLed = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        blinkinLed.setPattern(RevBlinkinLedDriver.BlinkinPattern.BREATH_GRAY);
    }

    @Override
    public void init_loop() {
        // Let operator work with wobble goal gripper during setup
        operatorControls.periodicTask();
    }

    @Override
    public void loop() {
        driverControls.periodicTask();
        operatorControls.periodicTask();

        totalCurrentSensor.update();

        double averageCurrent = totalCurrentSensor.getAverageCurrent();
        double maxCurrent = totalCurrentSensor.getMaxCurrent();

        telemetry.addData("C", "max: %.1fA avg: %.1fA", maxCurrent, averageCurrent);

        telemetry.update();
    }
}
