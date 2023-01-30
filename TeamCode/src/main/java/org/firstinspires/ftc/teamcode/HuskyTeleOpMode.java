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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
@Config
@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    // region DEFINE VARIABLES
    HuskyBot huskyBot = new HuskyBot();

    private ElapsedTime runtime = new ElapsedTime();
    final double END_GAME_TIME = 80.0;  // last 40 seconds
    final double FINAL_TIME = 110.0;    // last 10 seconds
    boolean endGameRumbled = false;
    boolean finalRumbled = false;

    double armSwivelPower = 0.0;
    double armExtendPower = 0.0;
    double armLiftPower = 0.0;

    private ElapsedTime finiteTimer = new ElapsedTime();
    public enum ArmState {
        ARM_WAIT,
        STEP_1,
        STEP_2,
        STEP_3,
        STEP_4
    }
    ArmState armState = ArmState.ARM_WAIT;

    int armLiftTargetPos;
    int armExtendTargetPos;
    double clawLiftTargetPos;
    boolean shouldChangeTheClawLift = false;
    // endregion

    private PIDController armUpPID;
    private PIDController armDownPID;
    public static double up_Kp = 0.0055, up_Ki = 0, up_Kd = 0.00025;
    public static double down_Kp = 0.00025, down_Ki = 0, down_Kd = 0.000005;
    public static double f = 0.045, l = 0;
    public static int target = 0;

    private final double ARM_LIFT_TICKS_PER_DEGREE = 28.0 * 5.23 * 5.23 * 3.61 / 360;

    // region DEFINE FUNCTIONS
    // method to smoothly accelerate a motor given a target velocity.
//    void smoothAcceleration(DcMotorEx motor, double targetVel, double accelRate) {
//        double currentVel = motor.getVelocity();
//        double changeVel = 0;
//
//        // check if currentVel is close to targetVel. if it is, set velocity directly to the target.
//        if (Math.abs(currentVel - targetVel) < accelRate) {
//            currentVel = targetVel;
//        }
//        else {
//            // if motor is decelerating (approaching 0 vel), increase deceleration rate.
//            if (Math.abs(currentVel) > Math.abs(targetVel)) {
//                accelRate *= 2;
//            }
//            // set +/- changeVel based on if currentVel is lower or higher than targetVel.
//            changeVel = (currentVel < targetVel) ? accelRate : -accelRate;
//        }
//
//        // change the velocity of the motor (accelerate) based on changeVel.
//        motor.setVelocity(currentVel + changeVel);
//    }
    // endregion

    @Override
    public void runOpMode() {

        // region INITIALIZATION
        huskyBot.init(hardwareMap);
        huskyBot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

//        telemetry.addData("Status", "Initialized");
//        telemetry.update();

        waitForStart();
        runtime.reset();

        huskyBot.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
        // endregion
        armUpPID = new PIDController(up_Kp, up_Ki, up_Kd);
        armDownPID = new PIDController(down_Kp, down_Ki, down_Kd);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // region TELE-OP LOOP
        while (opModeIsActive()) {

        // region CONTROLLER RUMBLE SIGNALS
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled) {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled) {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }
        // endregion


        // region DRIVE MECHANISMS
            Pose2d poseEstimate = huskyBot.drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());
            // uses the left trigger to dynamically shift between different drive speeds.
            // when the trigger is fully released, driveVelocity = 1.
            // when the trigger is fully pressed, driveVelocity = 0.2.
            double driveVelocity = (0.35 + 0.5 * gamepad1.left_trigger);

            huskyBot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * driveVelocity,
                            input.getY() * driveVelocity,
                            -gamepad1.right_stick_x * driveVelocity
                    )
            );
            huskyBot.drive.update();
        // endregion


        // region MANUAL ARM CONTROL
            // region Blocked by Preset FSM
            if (armState == ArmState.ARM_WAIT) {
                // Arm Lift Controls
                armUpPID.setPID(up_Kp, up_Ki, up_Kd);
                armDownPID.setPID(down_Kp, down_Ki, down_Kd);

                int armPos = huskyBot.armLiftMotor.getCurrentPosition();
                double pid;
                if (target >= armPos) {
                    pid = armUpPID.calculate(armPos, target);
                } else {
                    pid = armDownPID.calculate(armPos, target);
                }

                int extendPos = huskyBot.armExtendMotor.getCurrentPosition();
                double targetArmAngle = Math.toRadians((target - 470) / ARM_LIFT_TICKS_PER_DEGREE);
                double ff = (l * extendPos + 1) * f * Math.cos(targetArmAngle);

                armLiftPower = pid + ff;
                huskyBot.armLiftMotor.setPower(armLiftPower);

                telemetry.addData("extend ", extendPos);
                telemetry.addData("target angle ", targetArmAngle);
                telemetry.addData("current angle ", Math.toRadians((armPos - 470) / ARM_LIFT_TICKS_PER_DEGREE));
                telemetry.addData("target pos ", target);
                telemetry.addData("current pos ", armPos);
                telemetry.addData("pid ", pid);
                telemetry.addData("ff ", ff);
                telemetry.update();


                // Increases/Decreases Arm Length
                armExtendPower = gamepad2.dpad_up ? -ARM_EXTENSION_MAX_POWER : (gamepad2.dpad_down ? ARM_EXTENSION_MAX_POWER : 0);
                // Use Magnetic Limit Switches to limit extension of the arm.
                if (huskyBot.armExtendMin.isPressed()) {
                    armExtendPower = (armExtendPower > 0) ? 0 : armExtendPower;
                }
                if (huskyBot.armExtendMax.isPressed()) {
                    armExtendPower = (armExtendPower < 0) ? 0 : armExtendPower;
                }
                huskyBot.armExtendMotor.setPower(armExtendPower);


                // Claw Lift Servo Control
                if (gamepad2.right_stick_y != 0) {
                    huskyBot.servoMove(huskyBot.clawLift, -gamepad2.right_stick_y);
                }
            }
            // endregion

            // region Not Blocked by Preset FSM
            // Arm Swivel Controls
            armSwivelPower = -gamepad2.left_stick_x;
            armSwivelPower = Range.clip(armSwivelPower, -ARM_SWIVEL_MAX_POWER, ARM_SWIVEL_MAX_POWER);
            // Arm Swivel Limiters
            if (huskyBot.armSwivelMotor.getCurrentPosition() <= ARM_SWIVEL_RIGHT_LIMIT) {
                armSwivelPower = (armSwivelPower < 0) ? 0 : armSwivelPower;
            }
            if (huskyBot.armSwivelMotor.getCurrentPosition() >= ARM_SWIVEL_LEFT_LIMIT) {
                armSwivelPower = (armSwivelPower > 0) ? 0 : armSwivelPower;
            }
            huskyBot.armSwivelMotor.setPower(armSwivelPower);


            // Open/Close the Claw
            if (gamepad2.right_bumper || gamepad1.right_bumper) {
                huskyBot.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);
            }
            if (gamepad2.left_bumper || gamepad1.left_bumper) {
                huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
            }

            // Custom Claw Open/Close
            if (-gamepad2.right_trigger != 0) {
                huskyBot.servoMove(huskyBot.clawGrab, -gamepad2.right_trigger);
            }
            if (gamepad2.left_trigger != 0) {
                huskyBot.servoMove(huskyBot.clawGrab, gamepad2.left_trigger);
            }
            // endregion
        // endregion


        // region ARM PRESETS FINITE STATE MACHINE
            if(armState != ArmState.ARM_WAIT && (gamepad1.left_stick_button || gamepad1.right_stick_button || gamepad2.left_stick_button || gamepad2.right_stick_button)){
                huskyBot.armExtendMotor.setPower(0);
                huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                huskyBot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                huskyBot.armLiftMotor.setPower(ARM_LIFT_POWER_AT_REST);

                armState = ArmState.ARM_WAIT;
            }

            switch (armState){
                case ARM_WAIT:
                    finiteTimer.reset();
                    shouldChangeTheClawLift = false;

                    if(gamepad1.a) {
                        // Button A: Ground position
                        huskyBot.clawLift.setPosition(CLAW_LIFT_GROUND_POSITION);
                        huskyBot.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);

                        armLiftTargetPos = ARM_LIFT_GROUND_POSITION;
                        armExtendTargetPos = ARM_EXTEND_GROUND_POSITION;

                        armState = ArmState.STEP_1;
                    }
                    if(gamepad1.x){
                        // Button X: Low junction position
                        armLiftTargetPos = ARM_LIFT_LOW_POSITION;
                        armExtendTargetPos = ARM_EXTEND_LOW_POSITION;
                        clawLiftTargetPos = CLAW_LIFT_LOW_POSITION;

                        shouldChangeTheClawLift = true;

                        armState = ArmState.STEP_1;
                    }
                    if(gamepad1.b) {
                        // Button B: Medium junction position
                        armLiftTargetPos = ARM_LIFT_MED_POSITION;
                        armExtendTargetPos = ARM_EXTEND_MED_POSITION;
                        clawLiftTargetPos = CLAW_LIFT_MED_POSITION;

                        shouldChangeTheClawLift = true;

                        armState = ArmState.STEP_1;
                    }
                    if(gamepad1.y) {
                        // Button Y: High junction position
                        armLiftTargetPos = ARM_LIFT_HIGH_POSITION;
                        armExtendTargetPos = ARM_EXTEND_HIGH_POSITION;
                        clawLiftTargetPos = CLAW_LIFT_HIGH_POSITION;

                        shouldChangeTheClawLift = true;

                        armState = ArmState.STEP_1;
                    }

                    break;
                case STEP_1:
                    // Step 1: Reset the arm extender (close)

                    huskyBot.armExtendMotor.setTargetPosition(-10);
                    huskyBot.armExtendMotor.setPower(1.0);
                    huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    armState = ArmState.STEP_2;

                    break;
                case STEP_2:
                    // Step 2:
                    // Wait until the step 1 is completed
                    // Then change the arm lift's position (up or down based on the target position)

                    if(huskyBot.armExtendMotor.isBusy()){
                        if(finiteTimer.seconds() > 7){
                            gamepad1.rumble(1000);

                            huskyBot.armExtendMotor.setPower(0);
                            huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            armState = ArmState.ARM_WAIT;
                            break;
                        }

                        telemetry.addData("Arm State Status", "Arm is extending");
                    } else {
                        huskyBot.armExtendMotor.setPower(0);
                        huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        huskyBot.armLiftMotor.setTargetPosition(armLiftTargetPos);
                        huskyBot.armLiftMotor.setPower(0.35);
                        huskyBot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        armState = ArmState.STEP_3;
                    }

                    break;
                case STEP_3:
                    // Step 3:
                    // Wait until the step 2 is completed
                    // Then change the arm extender's position (in or out based on the target position)

                    if (huskyBot.armLiftMotor.isBusy()) {
                        if(finiteTimer.seconds() > 7){
                            gamepad1.rumble(1000);

                            huskyBot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                            huskyBot.armLiftMotor.setPower(ARM_LIFT_POWER_AT_REST);

                            armState = ArmState.ARM_WAIT;
                            break;
                        }

                        telemetry.addData("Arm State Status", "Arm lift is moving");
                    } else {
                        huskyBot.armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        huskyBot.armLiftMotor.setPower(ARM_LIFT_POWER_AT_REST);

                        huskyBot.armExtendMotor.setTargetPosition(armExtendTargetPos);
                        huskyBot.armExtendMotor.setPower(1.0);
                        huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                        armState = ArmState.STEP_4;
                    }


                    break;
                case STEP_4:
                    // Step 4:
                    // Wait until the step 3 is completed
                    // Then change armState to wait (default)

                    if (huskyBot.armExtendMotor.isBusy()) {
                        if(finiteTimer.seconds() > 7){
                            gamepad1.rumble(1000);

                            huskyBot.armExtendMotor.setPower(0);
                            huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                            armState = ArmState.ARM_WAIT;
                            break;
                        }

                        telemetry.addData("Arm State Status", "Arm is extending");
                    } else {
                        huskyBot.armExtendMotor.setPower(0);
                        huskyBot.armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                        if(shouldChangeTheClawLift){
                            if(finiteTimer.seconds() > 7){
                                gamepad1.rumble(1000);
                                armState = ArmState.ARM_WAIT;
                                break;
                            }

                            huskyBot.clawLift.setPosition(clawLiftTargetPos);
                        }

                        gamepad1.rumble(200);
                        armState = ArmState.ARM_WAIT;
                    }

                    break;

                default:
            }
        // endregion

/*
        // region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Stick", "y (%.2f), x (%.2f), rx (%.2f)", y, x, rx);
            telemetry.addData("Heading", poseEstimate.getHeading());
//            telemetry.addData("Actual Vel", "fl (%.2f), rl (%.2f)", huskyBot.frontLeftDrive.getVelocity(), huskyBot.rearLeftDrive.getVelocity());
//            telemetry.addData("Actual Vel", "fr (%.2f), rr (%.2f)", huskyBot.frontRightDrive.getVelocity(), huskyBot.rearRightDrive.getVelocity());
//            telemetry.addData("Target Vel", "fl (%.2f), rl (%.2f)", frontLeftVelocity, rearLeftVelocity);
//            telemetry.addData("Target Vel", "fr (%.2f), rr (%.2f)", frontRightVelocity, rearRightVelocity);
//            telemetry.addData("Power", "front left (%.2f), rear left (%.2f)", huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());
//            telemetry.addData("Power", "front right (%.2f), rear right (%.2f)", huskyBot.frontLeftDrive.getPower(), huskyBot.rearLeftDrive.getPower());

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
        // endregion
*/
        }
        // endregion

    }
}

