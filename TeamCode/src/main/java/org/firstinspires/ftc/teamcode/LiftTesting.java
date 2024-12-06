/* Copyright (c) 2023 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test - Lift", group = "Testing")
@Disabled
public class LiftTesting extends LinearOpMode {
    private DcMotor leftlift = null;
    private DcMotor rightlift = null;
    int liftHeightMax = 2200;
    int liftSafetyThreshold = 500;
    double liftPower = .8;
    String liftMode = "Encoder";

    @Override
    public void runOpMode() {

        double liftSafetyPowerOverride = 0;
        boolean liftSafetyOverride = false;
        int liftSafetyCheck = 0;
        int liftHeightRight = 0;
        int liftHeightLeft = 0;
        double liftPowerMin = 0.1;
        double effectivePower = 0;

        String control = "Left";
        String enabled = "Power Disabled";

        leftlift = hardwareMap.get(DcMotor.class, "left lift");
        rightlift = hardwareMap.get(DcMotor.class, "right lift");
        leftlift.setDirection(DcMotorSimple.Direction.REVERSE);
        // Initialize motors and encoders
        rightlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftlift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftlift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightlift.setTargetPosition(0);
        leftlift.setTargetPosition(0);
        rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            sleep(10);
            telemetry.addData(">", "Mode DPad L/R   Power DPAD U/D");
            telemetry.addLine()
                    .addData("Mode", liftMode)
                    .addData("Power", "%.2f", liftPower)
                    .addData("", enabled);
            telemetry.addData("L/R/B - X Y A:", control);
            telemetry.addData(">", "Cur L Height: %d / R Height %d", leftlift.getCurrentPosition(), rightlift.getCurrentPosition());
            telemetry.addData(">", "Tar L Height: %d / R Height %d", liftHeightLeft, liftHeightRight);
            telemetry.addLine()
                    .addData("Threshold", liftSafetyThreshold)
                    .addData("Check", liftSafetyCheck)
                    .addData("Power", "%.2f", (double) liftSafetyCheck / liftSafetyThreshold);

            telemetry.addData("Effective Power", effectivePower);

            telemetry.update();

            if (gamepad1.dpad_left) {
                liftMode = "Encoder";
                liftPower = liftPowerMin;
                liftHeightRight = rightlift.getCurrentPosition();
                liftHeightLeft = leftlift.getCurrentPosition();
                rightlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftlift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            } else if (gamepad1.dpad_right) {
                liftMode = "Power";
                liftPower = 0;
                rightlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                leftlift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            } else if (gamepad1.dpad_up) {
                liftPower = liftPower + .05;
                if (liftPower >= 1) {
                    liftPower = 1;
                }
            } else if (gamepad1.dpad_down) {
                liftPower = liftPower - .05;
                if (liftPower <= 0) {
                    liftPower = 0;
                }
            }
            if (gamepad1.left_bumper) enabled = "Power Disabled";
            else if (gamepad1.right_bumper) enabled = "Power Enabled";

            if (gamepad1.a) {
                control = "Left";
            } else if (gamepad1.b) {
                control = "Right";
            } else if (gamepad1.x) {
                control = "Both";
            }

            if (control == "Left" || control == "Both") {
                if (liftMode == "Encoder") {
                    liftHeightLeft = liftHeightLeft + (int) (-gamepad1.left_stick_y * 50);
                    if (liftHeightLeft > liftHeightMax) {
                        liftHeightLeft = liftHeightMax;
                    } else if (liftHeightLeft < 0) {
                        liftHeightLeft = 0;
                    }
                    if (control == "Both") {
                        liftHeightRight = liftHeightLeft;
                    }

                } else if (liftMode == "Power") {
                    liftPower = -gamepad1.left_stick_y;
                }

                // Check for maximum height
                liftSafetyCheck = liftHeightMax - leftlift.getCurrentPosition();

                if (liftSafetyCheck < liftSafetyThreshold) {
                    liftSafetyOverride = (liftMode == "Encoder" && liftHeightLeft > leftlift.getCurrentPosition())
                            || (liftMode == "Power" && liftPower > 0);
                    liftSafetyPowerOverride = (double) liftSafetyCheck / liftSafetyThreshold;
                } else if (leftlift.getCurrentPosition() < liftSafetyThreshold) {
                    liftSafetyOverride = (liftMode == "Encoder" && liftHeightLeft < leftlift.getCurrentPosition())
                            || (liftMode == "Power" && liftPower < 0);
                    liftSafetyPowerOverride = (double) -leftlift.getCurrentPosition() / liftSafetyThreshold;
                } else {
                    liftSafetyOverride = false;
                }

            }
            if (control == "Right") {
                if (liftMode == "Encoder") {
                    liftHeightRight = liftHeightRight + (int) (-gamepad1.right_stick_y * 50);
                } else if (liftMode == "Power") {
                    liftPower = -gamepad1.right_stick_y;
                }
            }

            if (enabled == "Power Enabled") {
                if (liftSafetyOverride) effectivePower = liftSafetyPowerOverride;
                else effectivePower = liftPower;

                leftlift.setTargetPosition(liftHeightLeft);
                rightlift.setTargetPosition(liftHeightRight);

                if (control == "Left") {
                    leftlift.setPower(effectivePower);
                    rightlift.setPower(0);
                } else if (control == "Right") {
                    leftlift.setPower(0);
                    rightlift.setPower(effectivePower);
                } else if (control == "Both") {
                    moveLift(liftHeightLeft, liftPower);
                }
            } else {
                leftlift.setPower(0);
                rightlift.setPower(0);
            }
        }
    }

    public void moveLift(int targetHeight, double liftPower) {

        double liftSafetyPowerOverride = 0;
        boolean liftSafetyOverride = false;
        int liftSafetyCheck = 0;
        double effectivePower = 0;

        if (targetHeight > liftHeightMax) targetHeight = liftHeightMax;
        else if (targetHeight < 0) targetHeight = 0;

        // Check for safety thresholds
        liftSafetyCheck = liftHeightMax - leftlift.getCurrentPosition();

        if (liftSafetyCheck < liftSafetyThreshold) {
            liftSafetyOverride = (liftMode == "Encoder" && targetHeight > leftlift.getCurrentPosition())
                    || (liftMode == "Power" && liftPower > 0);
            liftSafetyPowerOverride = (double) liftSafetyCheck / liftSafetyThreshold;
        } else if (leftlift.getCurrentPosition() < liftSafetyThreshold) {
            liftSafetyOverride = (liftMode == "Encoder" && targetHeight < leftlift.getCurrentPosition())
                    || (liftMode == "Power" && liftPower < 0);
            liftSafetyPowerOverride = (double) -leftlift.getCurrentPosition() / liftSafetyThreshold;
        } else {
            liftSafetyOverride = false;
        }

        if (liftSafetyOverride) effectivePower = liftSafetyPowerOverride;
        else effectivePower = liftPower;

        leftlift.setPower(effectivePower);
        rightlift.setPower(effectivePower);

        leftlift.setTargetPosition(targetHeight);
        rightlift.setTargetPosition(targetHeight);
    }
}
