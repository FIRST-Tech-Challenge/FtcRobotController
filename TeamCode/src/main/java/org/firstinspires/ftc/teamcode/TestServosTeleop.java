/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test Servos Teleop", group="Linear Opmode")
//@Disabled
public class TestServosTeleop extends LinearOpMode {
    private ServoFunctions sf = null;
    @Override
    public void runOpMode() {
        sf = new ServoFunctions(this);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            if (currentGamepad1.left_bumper)
                sf.MoveElbowRelative(-0.01);
            if (currentGamepad1.right_bumper)
                sf.MoveElbowRelative(0.01);

            if (currentGamepad1.right_trigger > 0.5)
                sf.MoveShoulderRelative(0.01);

            if (currentGamepad1.left_trigger > 0.5)
                sf.MoveShoulderRelative(-0.01);

            if (currentGamepad1.y)
                sf.MoveClawRelative(0.01);

            if (currentGamepad1.a)
                sf.MoveClawRelative(-0.01);

            if (currentGamepad1.b)
                sf.MovePixelReleaseServoRelative(0.01);

            if (currentGamepad1.x)
                sf.MovePixelReleaseServoRelative(-0.01);

            if (currentGamepad1.start)
                sf.PutPixelInBackBoard();

            telemetry.addData("Pixel Release Servo Position", "%6.4f", sf.GetPixelReleaseServoPosition());
            telemetry.addData("Shoulder Servo Position", "%6.4f", sf.GetShoulderServoPosition());
            telemetry.addData("Elbow Servo Position", "%6.4f", sf.GetElbowServoPosition());
            telemetry.addData("Claw Servo Position", "%6.4f", sf.GetClawServoPosition());
            telemetry.update();
        }
    }
}

