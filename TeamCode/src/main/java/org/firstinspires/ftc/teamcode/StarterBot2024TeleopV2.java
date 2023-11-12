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
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Starter Bot V2", group="Iterative Opmode")

public class StarterBot2024TeleopV2 extends OpMode
{
    RevStarterRobotHardware robot       = new RevStarterRobotHardware(this);

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private boolean manualMode = false;
    private final double armManualDeadband = 0.03;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        robot.init();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();

        robot.resetArmEncoders();
        robot.gotoHomePosition();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double manualArmPower;

        //DRIVE
        double drive = -gamepad1.left_stick_y/5.0;
        double turn  =  gamepad1.right_stick_x/5.0;
        robot.driveRobot(drive, turn);

        //ARM & WRIST
        manualArmPower = gamepad1.right_trigger - gamepad1.left_trigger;
        if (Math.abs(manualArmPower) > armManualDeadband) {
            if (!manualMode) {
                robot.setArmsManualMode();
                manualMode = true;
            }
            robot.moveArms(manualArmPower);
        }
        else {
            if (manualMode) {
                robot.setArmsPresetMode();
                manualMode = false;
            }

            //preset buttons
            if (gamepad1.a) {
                robot.gotoHomePosition();
            }
            else if (gamepad1.b) {
                robot.gotoIntakePosition();
            }
            else if (gamepad1.y) {
                robot.gotoScorePosition();
            }
        }

        //Re-zero encoder button
        if (gamepad1.start) {
            robot.resetArmEncoders();
            manualMode = false;
        }

        //Watchdog to shut down motor once the arm reaches the home position
        if (!manualMode){
            robot.watchdog();
        }

        //GRIPPER
        if (gamepad1.left_bumper || gamepad1.right_bumper) {
            robot.openGrip();
        }
        else {
            robot.closeGrip();
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Gamepad", "drive (%.2f), turn (%.2f)", drive, turn);
        telemetry.addData("Manual Power", manualArmPower);

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

} 