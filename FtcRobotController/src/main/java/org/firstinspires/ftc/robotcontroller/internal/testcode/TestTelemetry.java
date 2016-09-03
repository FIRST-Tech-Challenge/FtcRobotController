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
package org.firstinspires.ftc.robotcontroller.internal.testcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.TelemetryInternal;

import java.util.Locale;

/**
 * {@link TestTelemetry} is a simple test of telemetry.
 */
@Autonomous(name = "Test Telemetry", group = "Tests")
@Disabled
public class TestTelemetry extends LinearOpMode
    {
    @Override public void runOpMode() throws InterruptedException
        {
        waitForStart();

        testNonAutoClear();
        testLogDashboardInteraction();

        while (opModeIsActive())
            {
            idle();
            }
        }

    void testNonAutoClear() throws InterruptedException
        {
        ((TelemetryInternal)telemetry).resetTelemetryForOpMode();

        telemetry.setAutoClear(false);

        Telemetry.Item counting = telemetry.addData("counting", 1).setRetained(true);
        telemetry.addData("unretained", "red");
        telemetry.addData("retained", new Func<String>() {
            @Override public String value() {
                return String.format(Locale.getDefault(), "%dms", System.currentTimeMillis());
            }});
        telemetry.addData("unretained", "green");
        telemetry.update();
        delay();

        counting.setValue(2);
        telemetry.update();
        delay();

        counting.setValue(3);
        telemetry.update();
        delay();

        telemetry.clear();
        delay();

        counting.setValue(4);
        telemetry.update();
        delay();
        }

    void testLogDashboardInteraction() throws InterruptedException
        {
        ((TelemetryInternal)telemetry).resetTelemetryForOpMode();

        int count = 0;

        telemetry.log().add("one");
        delay();

        telemetry.addData("count", count++);
        delay();

        telemetry.addData("count", count++);
        delay();

        telemetry.log().add("two");
        delay();

        telemetry.update();
        delay();

        telemetry.log().add("three (should see count 0 & 1)");
        delay();

        telemetry.addData("count", count++);
        delay();

        telemetry.log().add("four (should see count 0 & 1)");
        delay();

        telemetry.update();
        delay();

        telemetry.log().add("five (should see count 2)");
        delay();
        }

    void delay() throws InterruptedException
        {
        Thread.sleep(2000);
        }

    }
