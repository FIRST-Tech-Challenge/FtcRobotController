/*
 * MIT License
 *
 * Copyright (c) 2024 ParkCircus Productions; All Rights Reserved
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.hello;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/**
 * {@link HelloAuto} is a basic autonomous OpMode used for initial robot testing.
 * This class serves as a template and a guide for how to properly document a new OpMode
 * for the FTC Robot Controller SDK.
 *
 * <p>It demonstrates the fundamental lifecycle of a LinearOpMode, including
 * initialization, waiting for start, and running a main control loop. The use of
 * telemetry for debugging is also highlighted.</p>
 *
 * @author Matha Goram
 * @version 1.1
 * @since 2024-05-20
 *
 * @see LinearOpMode
 * @see com.qualcomm.robotcore.eventloop.opmode.TeleOp
 * @see <a href="http://ftc-docs.firstinspires.org/en/latest/programming_resources/linear_op_mode/linear-op-mode.html">FIRST Linear OpMode Documentation</a>
 */
@Autonomous(name="Hello Auto", group="Hello", preselectTeleOp="HelloTeleOp")
// @Disabled
public class HelloAuto extends LinearOpMode {

    /**
     * The entry point for the OpMode. This method is called once when the
     * OpMode is initialized and will block until the OpMode is stopped or
     * its execution completes.
     *
     * <p>This is where all robot hardware is initialized and the main
     * robot logic is executed.</p>
     */
    @Override
    public void runOpMode() {

        // Use telemetry to send data to the Driver Station.
        // This is a crucial step for debugging and providing user feedback.
        // The telemetry data is buffered until telemetry.update() is called.
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // The waitForStart() method pauses the OpMode until the user
        // presses the "Play" button on the Driver Station. This allows
        // the driver to prepare the robot before the autonomous period begins.
        waitForStart();

        // Record the start time of the OpMode in milliseconds.
        long startTime = System.currentTimeMillis();

        // The opModeIsActive() method returns true if the OpMode is running and the robot
        // is not stopped. This is the primary condition for a main control loop.
        if (opModeIsActive()) {

            // This is the main loop for the OpMode. It runs continuously
            // until the OpMode is stopped (e.g., by the user pressing "Stop").
            while (opModeIsActive()) {

                // Calculate the time elapsed since the OpMode started.
                long elapsedTime = System.currentTimeMillis() - startTime;

                // Add a timestamp to the telemetry. The '%.2f' format
                // displays the time in seconds with two decimal places.
                telemetry.addData("Elapsed Time", "%.2f seconds", (double)elapsedTime / 1000.0);

                // Add messages to the telemetry. The "Status" message
                // provides a real-time status of the OpMode's execution.
                telemetry.addData("Status", "Running");
                telemetry.addData("Message", "Hello, Auto World!");
                telemetry.addData("N.B.", "Sleep is 1 sec but other tasks need time too!");
                telemetry.update();

                // The sleep() method is used to pause the OpMode for a specified
                // duration (in milliseconds). This is important to prevent the
                // OpMode from consuming too much processing power, allowing other
                // tasks (like telemetry updates) to occur smoothly.
                sleep(1000);
            }
        }
    }
}

