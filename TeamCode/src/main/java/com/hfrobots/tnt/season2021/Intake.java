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

import com.google.common.base.Ticker;
import com.hfrobots.tnt.corelib.drive.StallDetector;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.TimeUnit;

public class Intake {
    public static final float INTAKE_POWER = 1;

    public static final float OUTTAKE_POWER = -1;

    // NR-3.7 Orbital: positive voltage rotation CCW, encoder/sec 3400
    private final DcMotorEx intakeMotor;

    private StallDetector stallDetector;

    private final Ticker ticker;

    public Intake(HardwareMap hardwareMap, Ticker ticker){
        intakeMotor = hardwareMap.get(DcMotorEx.class,"intakeMotor");
        this.ticker = ticker;
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        resetStallDetector();
    }

    public void resetStallDetector() {
        stallDetector = new StallDetector(ticker, 5, TimeUnit.SECONDS.toMillis(1));
    }

    public void intake(float speed){
        intakeMotor.setPower(INTAKE_POWER * Math.abs(speed));
    }

    public void outtake(float speed){
        intakeMotor.setPower(OUTTAKE_POWER * Math.abs(speed));
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public boolean isIntaking() {
        return Math.signum(intakeMotor.getPower()) == Math.signum(INTAKE_POWER);
    }

    public boolean isOuttaking() {
        return Math.signum(intakeMotor.getPower()) == Math.signum(OUTTAKE_POWER);
    }

    public boolean isStalled() {
        return stallDetector.isStalled(intakeMotor.getCurrentPosition());
    }
}
