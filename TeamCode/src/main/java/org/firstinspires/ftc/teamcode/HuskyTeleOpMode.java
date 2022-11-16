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

import static org.firstinspires.ftc.teamcode.HuskyBot.ARM_EXTENSION_MAX_POWER;
import static org.firstinspires.ftc.teamcode.HuskyBot.ARM_LIFT_MAX_POWER;
import static org.firstinspires.ftc.teamcode.HuskyBot.ARM_LIFT_POWER_AT_REST;
import static org.firstinspires.ftc.teamcode.HuskyBot.ARM_SWIVEL_MAX_POWER;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    HuskyBot huskyBot = new HuskyBot();

    final double END_GAME_TIME = 80.0;  // last 40 seconds
    final double FINAL_TIME = 110.0;    // last 10 seconds
    boolean endGameRumbled = false;
    boolean finalRumbled = false;

    double armSwivelPower = 0.0;
    double armExtendPower = 0.0;
    double armLiftPower = 0.0;
    double clawRotationPosition = 0.0;
    double clawLiftPosition = 0.0;

    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double y, x, rx;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled)  {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled)  {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }

            // drive mechanism
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            double frontLeftVelocity = (y + x + rx) * HuskyBot.VELOCITY_CONSTANT;
            double rearLeftVelocity = (y - x + rx) * HuskyBot.VELOCITY_CONSTANT;
            double frontRightVelocity = (y - x - rx) * HuskyBot.VELOCITY_CONSTANT;
            double rearRightVelocity = (y + x - rx) * HuskyBot.VELOCITY_CONSTANT;

            // apply the calculated values to the motors.
            huskyBot.frontLeftDrive.setVelocity(frontLeftVelocity);
            huskyBot.rearLeftDrive.setVelocity(rearLeftVelocity);
            huskyBot.frontRightDrive.setVelocity(frontRightVelocity);
            huskyBot.rearRightDrive.setVelocity(rearRightVelocity);

            // arm/claw mechanisms
            // todo + IMPORTANT: we will have to limit this to rotate only 240 degrees once the arm is added.
            armSwivelPower = -gamepad2.left_stick_x;
            armSwivelPower = Range.clip(armSwivelPower, -ARM_SWIVEL_MAX_POWER, ARM_SWIVEL_MAX_POWER);
            huskyBot.armSwivel.setPower(armSwivelPower);

            armLiftPower = gamepad2.left_stick_y;
            // todo: this will have to be tuned for stability
            armLiftPower = Range.clip(armLiftPower, -ARM_LIFT_MAX_POWER, ARM_LIFT_MAX_POWER);
            if (armLiftPower == 0) {
                huskyBot.armLift.setPower(ARM_LIFT_POWER_AT_REST);
            }
            else {
                huskyBot.armLift.setPower(armLiftPower);
            }

            // Increases/Decreases Arm Length
            armExtendPower = gamepad2.dpad_up ? ARM_EXTENSION_MAX_POWER : (gamepad2.dpad_down ? -ARM_EXTENSION_MAX_POWER : 0);
            huskyBot.armExtend.setPower(armExtendPower);

            clawRotationPosition = gamepad2.right_stick_x;
            huskyBot.clawRotate.setPosition(clawRotationPosition);

            clawLiftPosition = gamepad2.right_stick_y;
            huskyBot.clawLift.setPosition(clawLiftPosition);

            if (gamepad2.x) {
                huskyBot.clawGrab.setPosition(0.3);
            }
            if (gamepad2.a) {
                huskyBot.clawGrab.setPosition(0.0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Stick", "y (%.2f), x (%.2f), rx (%.2f)", y, x, rx);
            telemetry.addData("Actual Vel", "fl (%.2f), rl (%.2f)",
                    huskyBot.frontLeftDrive.getVelocity(), huskyBot.rearLeftDrive.getVelocity());
            telemetry.addData("Actual Vel", "fr (%.2f), rr (%.2f)",
                    huskyBot.frontRightDrive.getVelocity(), huskyBot.rearRightDrive.getVelocity());
            telemetry.addData("Target Vel", "fl (%.2f), rl (%.2f)", frontLeftVelocity, rearLeftVelocity);
            telemetry.addData("Target Vel", "fr (%.2f), rr (%.2f)", frontRightVelocity, rearRightVelocity);
            telemetry.addData("Power", "front left (%.2f), rear left (%.2f)", huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());
            telemetry.addData("Power", "front right (%.2f), rear right (%.2f)", huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());

            // Show the Arm/Claw Telemetry
            telemetry.addData("Arm Swivel", "Power: (%.2f), Pos: (%d)",
                    huskyBot.armSwivel.getPower(), huskyBot.armSwivel.getCurrentPosition());
            telemetry.addData("Arm Lift", "Power: (%.2f), Pos: (%d)",
                    huskyBot.armLift.getPower(), huskyBot.armLift.getCurrentPosition());
            telemetry.addData("Arm Extend", "Power: (%.2f), Pos: (%d)",
                    huskyBot.armExtend.getPower(), huskyBot.armExtend.getCurrentPosition());
            telemetry.addData("Claw Rotate", "Pos: (%.2f)", huskyBot.clawRotate.getPosition());
            telemetry.addData("Claw Lift", "Pos: (%.2f)", huskyBot.clawLift.getPosition());
            telemetry.addData("Claw Grab", "Pos: (%.2f)", huskyBot.clawGrab.getPosition());

            telemetry.update();
        }
    }
}