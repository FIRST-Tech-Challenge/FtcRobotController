/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.DummyBot;
import org.firstinspires.ftc.teamcode.bots.OutreachBot;

//Opmode for quick testing of motors
@TeleOp(name="Dummy", group="Robot15173")
@Disabled
public class DummyMode extends LinearOpMode{

    // Declare OpMode members.
    DummyBot robot = new DummyBot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this.hardwareMap, telemetry);
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();


            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double drive1 = gamepad1.left_stick_y;
                robot.moveMotor1(drive1);

                double drive2 = gamepad1.right_stick_y;
                robot.moveMotor2(drive2);

                double drive3 = gamepad2.left_stick_y;
                robot.moveMotor3(drive3);

                double drive4 = gamepad2.right_stick_y;
                robot.moveMotor4(drive4);

            }
        }

        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
    }
}
