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
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;
import org.firstinspires.ftc.teamcode.skills.BotThreadAction;
import org.firstinspires.ftc.teamcode.skills.RingDetector;

// Main Op Mode
@TeleOp(name = "Ultimate", group = "Robot15173")
public class UltimateMode extends LinearOpMode {

    // Declare OpMode members.
    UltimateBot robot = new UltimateBot();
    private ElapsedTime runtime = new ElapsedTime();
    boolean changedclaw = true;
    boolean changedintake = false;
    boolean changedshooter = false;
    boolean intakeReverse = false;
    boolean buttonpressable = true;
    double delaytime = 200;
    double startdelay = 0;
    double grabdelay = 0;
    private BotThreadAction bta = null;
    Thread btaThread = null;

    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.setDriveToPowerMode();
            } catch (Exception ex) {
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
                if (ltrigger > 0) {
                    turn = -ltrigger;
                } else if (rtrigger > 0) {
                    turn = rtrigger;
                }

                double strafe = gamepad1.right_stick_x;

                buttonpressable = ((runtime.milliseconds() - startdelay) >= delaytime);

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
                if (gamepad1.dpad_right && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedclaw = !changedclaw;
                }

                if (changedclaw) {
                    robot.closeWobbleClaw();
                } else {
                    robot.openWobbleClaw();
                }

                // move swing thread
//                if (gamepad1.dpad_up) {
//                    bta = new BotThreadAction(robot, telemetry, "wobbleforward", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.dpad_down) {
//                    bta = new BotThreadAction(robot, telemetry, "wobbleback", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.dpad_left) {
//                    bta = new BotThreadAction(robot, telemetry, "wobblewall", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                } else if (gamepad1.x && buttonpressable) {
//                    startdelay = runtime.milliseconds();
//                    bta = new BotThreadAction(robot, telemetry, "wallclose", this);
//                    btaThread = new Thread(bta);
//                    btaThread.start();
//                    changedclaw = !changedclaw;
//                }


                // wobble swing regular
                if (gamepad1.dpad_up && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.forwardWobbleSwing();
                } else if (gamepad1.dpad_down && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.backWobbleSwing();
                } else if (gamepad1.dpad_left) {
                    robot.wobbleLittleUp();
                } else if (gamepad1.x && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.liftWallGrab();
                    changedclaw = !changedclaw;
                } else if (gamepad1.y) {
                    robot.wobbleLittleDown();
                }

                // move intake
                if (gamepad1.a && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedintake = !changedintake;
                }

                if (changedintake) {
                    robot.intake();
                } else {
                    robot.stopintake();
                }

                if (gamepad1.b && buttonpressable){
                    startdelay = runtime.milliseconds();
                    intakeReverse = !intakeReverse;
                }

                if (intakeReverse){
                    robot.intakeReverse();
                }

                // move shooter
                if (gamepad1.left_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    changedshooter = !changedshooter;
                }

                if (changedshooter) {
                    robot.shooter();
                } else {
                    robot.stopshooter();
                }

                // shoot with servo
                if (gamepad1.right_bumper && buttonpressable) {
                    startdelay = runtime.milliseconds();
                    robot.shootServo();
                }

                telemetry.addData("Left", robot.getLeftOdometer());
                telemetry.addData("Right", robot.getRightOdometer());
                telemetry.addData("Hor", robot.getHorizontalOdometer());
                telemetry.addData("Wobble Position", robot.getWobblePos());
                telemetry.update();
            }
        } catch (Exception ex) {
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
            sleep(10000);
        }
        finally {
            robot.stopintake();
            robot.stopshooter();
        }
    }
}
