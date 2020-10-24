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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.DummyBot;
import org.firstinspires.ftc.teamcode.bots.SwingPosition;
import org.firstinspires.ftc.teamcode.bots.UltimateBot;

//Opmode for quick testing of motors
@TeleOp(name="Ultimate", group="Robot15173")
public class UltimateMode extends LinearOpMode{

    // Declare OpMode members.
    UltimateBot robot = new UltimateBot();
    private ElapsedTime runtime = new ElapsedTime();
    private boolean intakeon = false;
    boolean changedclaw = false;
    boolean changedintake = false;

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this,this.hardwareMap, telemetry);
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
                double drive = gamepad1.left_stick_y;
                double turn = 0;
                double ltrigger = gamepad1.left_trigger;
                double rtrigger = gamepad1.right_trigger;
                if (ltrigger > 0){
                    turn = -ltrigger;
                }
                else if (rtrigger > 0){
                    turn = rtrigger;
                }

                double strafe = gamepad1.right_stick_x;

                if (Math.abs(strafe) > 0) {
                    telemetry.addData("Strafing", "Left: %2f", strafe);
                    telemetry.update();
                    if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

                // move claw
                if(!changedclaw){
                    if(gamepad1.dpad_right){
                        changedclaw = true;

                        robot.openWobbleClaw();
                    }
                }else {
                    if(gamepad1.dpad_right){
                        changedclaw = false;

                        robot.closeWobbleClaw();
                    }
                }

//                if (gamepad1.left_bumper){
//                    robot.closeWobbleClaw();
//                }
//                else if (gamepad1.right_bumper) {
//                    robot.openWobbleClaw();
//                }

                if (gamepad1.dpad_up) {
                    robot.forwardWobbleSwing();
                }
                else if (gamepad1.dpad_down) {
                    robot.backWobbleSwing();
                }
                else if (gamepad1.dpad_left) {
                    robot.liftAndHoldWobbleSwing();
                }

                // move intake
                if(!changedintake){
                    if(gamepad1.a){
                        changedintake = true;
                        robot.intake();
                    }
                }else {
                    if(gamepad1.a){
                        changedintake = false;
                        robot.stopintake();
                    }
                }

                telemetry.addData("Heading", robot.getGyroHeading());
                telemetry.addData("Horiz encoder", robot.getHorizontalOdometer());
                telemetry.update();
            }
        }

        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep (10000);
        }
    }
}
