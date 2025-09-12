/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Demonstrates how to use an OpMode to store data in the blackboard and retrieve it.
 * This is useful for storing information in Auto and then retrieving it in your TeleOp.
 *
 * Be aware that this is NOT saved across power reboots or downloads or robot resets.
 *
 * You also need to make sure that both the storing and retrieving are done using the same
 * type.   For example, storing a Double in the blackboard and retrieving it as an Integer
 * will fail when you typecast it.
 *
 * The blackboard is a standard hashmap so you can use methods like:
 * put, get, containsKey, remove, etc.
 */
@TeleOp(name = "Concept: Blackboard", group = "Concept")
@Disabled
public class ConceptBlackboard extends OpMode {
    // We use constants here so we won't have the problem of typos in different places when we
    // meant to refer to the same thing.
    public static final String TIMES_STARTED_KEY = "Times started";
    public static final String ALLIANCE_KEY = "Alliance";

    /**
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        // This gets us what is in the blackboard or the default if it isn't in there.
        Object timesStarted = blackboard.getOrDefault(TIMES_STARTED_KEY, 0);
        blackboard.put(TIMES_STARTED_KEY, (int) timesStarted + 1);

        telemetry.addData("OpMode started times", blackboard.get(TIMES_STARTED_KEY));
    }

    /**
     * This method will be called repeatedly during the period between when
     * the START button is pressed and when the OpMode is stopped.
     * <p>
     * If the left bumper is pressed it will store the value "RED" in the blackboard for alliance.
     * If the right bumper is pressed it will store the value "BLUE" in the blackboard for alliance.
     * <p>
     * You'll notice that if you quit the OpMode and come back in, the value will persist.
     */
    @Override
    public void loop() {
        if (gamepad1.left_bumper) {
            blackboard.put(ALLIANCE_KEY, "RED");
        } else if (gamepad1.right_bumper) {
            blackboard.put(ALLIANCE_KEY, "BLUE");
        }
        telemetry.addData("Alliance", blackboard.get(ALLIANCE_KEY));
    }
}
