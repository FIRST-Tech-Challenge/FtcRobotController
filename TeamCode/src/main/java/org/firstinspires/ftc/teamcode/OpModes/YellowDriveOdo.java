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

import android.graphics.Point;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.bots.YellowBot;
import org.firstinspires.ftc.teamcode.odometry.RobotCoordinatePosition;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="YellowDriveOdo", group="Robot15173")
@Disabled
public class YellowDriveOdo extends LinearOpMode {

    // Declare OpMode members.
    YellowBot robot   = new YellowBot();
    private ElapsedTime     runtime = new ElapsedTime();
    RobotCoordinatePosition locator = null;


    @Override
    public void runOpMode() {
        try {
            try {
                robot.init(this, this.hardwareMap, telemetry);
                robot.initGyro();
                robot.initCalibData();
            }
            catch (Exception ex){
                telemetry.addData("Init", ex.getMessage());
            }

            telemetry.update();

            locator = new RobotCoordinatePosition(robot, new Point(30, 24), 0,RobotCoordinatePosition.THREAD_INTERVAL);
            locator.reverseHorEncoder();
            Thread positionThread = new Thread(locator);
            positionThread.start();

            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {

                // POV Mode uses left stick to go forward, and right stick to turn.
                // - This uses basic math to combine motions and is easier to drive straight.
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
                      if (strafe < 0) {
                        robot.strafeRight(Math.abs(strafe));
                    } else {
                        robot.strafeLeft(Math.abs(strafe));
                    }
                } else {
                    robot.move(drive, turn);
                }

                // get gyroscope data
                telemetry.addData("X ", locator.getXInches() );
                telemetry.addData("Y ", locator.getYInches() );
                telemetry.addData("Orientation (Degrees)", locator.getOrientation());
                telemetry.addData("Hor Odo", robot.getHorizontalOdometer());
                telemetry.addData("L Odo", robot.getLeftOdometer());
                telemetry.addData("R Odo", robot.getRightOdometer());

                telemetry.update();

                telemetry.update();
            }
        }
        catch (Exception ex){
            telemetry.addData("Issues with the OpMode", ex.getMessage());
            telemetry.update();
        }
        finally {
           if (locator != null){
               locator.stop();
           }
        }
    }
}
