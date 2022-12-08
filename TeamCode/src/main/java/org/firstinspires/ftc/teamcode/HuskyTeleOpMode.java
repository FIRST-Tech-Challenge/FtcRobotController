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

import static org.firstinspires.ftc.teamcode.HuskyBot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    final double END_GAME_TIME = 80.0;  // last 40 seconds
    final double FINAL_TIME = 110.0;    // last 10 seconds
    HuskyBot huskyBot = new HuskyBot();
    boolean endGameRumbled = false;
    boolean finalRumbled = false;
    double armSwivelPower = 0.0;
    double armExtendPower = 0.0;
    double armLiftPower = 0.0;
    double armLiftPowerDivider = 4;

    double clawLevel = 0.9;

    private ElapsedTime runtime = new ElapsedTime();

    // method to smoothly accelerate a motor given a target velocity.
    void smoothAcceleration(DcMotorEx motor, double targetVel, double accelRate) {
        double currentVel = motor.getVelocity();
        double changeVel = 0;

        // check if currentVel is close to targetVel. if it is, set velocity directly to the target.
        if (Math.abs(currentVel - targetVel) < accelRate) {
            currentVel = targetVel;
        }
        else {
            // if motor is decelerating (approaching 0 vel), increase deceleration rate.
            if (Math.abs(currentVel) > Math.abs(targetVel)) {
                accelRate *= 2;
            }
            // set +/- changeVel based on if currentVel is lower or higher than targetVel.
            changeVel = (currentVel < targetVel) ? accelRate : -accelRate;
        }

        // change the velocity of the motor (accelerate) based on changeVel.
        motor.setVelocity(currentVel + changeVel);
    }

    /*public void bReset()
    {
      Get Encoder
    }*/


    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        double y, x, rx;

        huskyBot.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled) {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled) {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }

            // drive mechanism
            y = -gamepad1.left_stick_y; // Remember, this is reversed!
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            // uses the left trigger to dynamically shift between different drive speeds.
            // when the trigger is fully released, driveVelocity = 1.
            // when the trigger is fully pressed, driveVelocity = 0.2.
            float driveVelocity = (float) (1 - 0.8 * gamepad1.left_trigger);

            // calculate motor velocities.
            double frontLeftVelocity = (y + x + rx) * driveVelocity * HuskyBot.VELOCITY_CONSTANT;
            double rearLeftVelocity = (y - x + rx) * driveVelocity * HuskyBot.VELOCITY_CONSTANT;
            double frontRightVelocity = (y - x - rx) * driveVelocity * HuskyBot.VELOCITY_CONSTANT;
            double rearRightVelocity = (y + x - rx) * driveVelocity * HuskyBot.VELOCITY_CONSTANT;

            // apply the calculated values to the motors using smooth acceleration.
            smoothAcceleration(huskyBot.frontLeftDrive, frontLeftVelocity, HuskyBot.VELOCITY_CONSTANT/5);
            smoothAcceleration(huskyBot.rearLeftDrive, rearLeftVelocity, HuskyBot.VELOCITY_CONSTANT/5);
            smoothAcceleration(huskyBot.frontRightDrive, frontRightVelocity, HuskyBot.VELOCITY_CONSTANT/5);
            smoothAcceleration(huskyBot.rearRightDrive, rearRightVelocity, HuskyBot.VELOCITY_CONSTANT/5);

            // arm/claw mechanisms
            // todo + IMPORTANT: we will have to limit this to rotate only 240 degrees once the arm is added.
            armSwivelPower = -gamepad2.left_stick_x;
            armSwivelPower = Range.clip(armSwivelPower, -ARM_SWIVEL_MAX_POWER, ARM_SWIVEL_MAX_POWER);
            huskyBot.armSwivelMotor.setPower(armSwivelPower);

            if(gamepad2.left_stick_y > 0){
                armLiftPowerDivider = 5.8;
            } else{
                if(huskyBot.armLiftMotor.getCurrentPosition() < 350){
                    armLiftPowerDivider = 4;
                } else {
                    armLiftPowerDivider = 3.1;
                }
            }

            armLiftPower = -gamepad2.left_stick_y/armLiftPowerDivider;
            armLiftPower = Range.clip(armLiftPower, -ARM_LIFT_MIN_POWER, ARM_LIFT_MAX_POWER);
            if(huskyBot.armLiftMotor.getCurrentPosition() < ARM_LIFT_MAX_POSITION)
            {
                if (armLiftPower == 0) {
                    huskyBot.armLiftMotor.setPower(ARM_LIFT_POWER_AT_REST);
                }
//            else if (armLiftPower < 0) {
//                huskyBot.armLiftMotor.setPower(ARM_LIFT_MIN_POWER);
//            }
                else {
                    huskyBot.armLiftMotor.setPower(armLiftPower);
                }
            }
            else
            {
                huskyBot.armLiftMotor.setPower(0);
            }

            //equilibrium code
            if(gamepad2.right_stick_y == 0){
                huskyBot.clawLift.setPosition(((huskyBot.armLiftMotor.getCurrentPosition() * -0.89)/ARM_LIFT_MAX_POSITION) + clawLevel);
            }
            if(gamepad2.right_stick_y > 0){
                clawLevel += 0.1;
            } else{
                clawLevel -=0.1;
            }

            // Increases/Decreases Arm Length
            armExtendPower = gamepad2.dpad_up ? -ARM_EXTENSION_MAX_POWER : (gamepad2.dpad_down ? ARM_EXTENSION_MAX_POWER : 0);
            huskyBot.armExtendMotor.setPower(armExtendPower);

            if (gamepad2.right_stick_y != 0) {
                huskyBot.servoMove(huskyBot.clawLift, -gamepad2.right_stick_y);
            }

            if (gamepad2.x) {
                huskyBot.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);
            }
            if (gamepad2.a) {
                huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
            }
            if (-gamepad2.right_trigger != 0) {
                huskyBot.servoMove(huskyBot.clawGrab, -gamepad2.right_trigger);
            }
            if (gamepad2.left_trigger != 0) {
                huskyBot.servoMove(huskyBot.clawGrab, gamepad2.left_trigger);
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
                    huskyBot.armSwivelMotor.getPower(), huskyBot.armSwivelMotor.getCurrentPosition());
            telemetry.addData("Arm Lift", "Left Y: (%.2f), Power: (%.2f), Pos: (%d)",
                    gamepad2.left_stick_y, huskyBot.armLiftMotor.getPower(), huskyBot.armLiftMotor.getCurrentPosition());
            telemetry.addData("Arm Extend", "Power: (%.2f), Pos: (%d)",
                    huskyBot.armExtendMotor.getPower(), huskyBot.armExtendMotor.getCurrentPosition());
            telemetry.addData("Claw Lift", "Right Y: (%.2f), Pos: (%.2f)",
                    gamepad2.right_stick_y, huskyBot.clawLift.getPosition());
            telemetry.addData("Claw Grab", "Pos: (%.2f)", huskyBot.clawGrab.getPosition());
            telemetry.addData("Arm Lift Power Divider", armLiftPowerDivider);
            telemetry.update();
        }
    }
}

