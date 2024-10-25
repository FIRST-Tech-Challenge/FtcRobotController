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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Mecanum Driver", group="TeleOp")
public class MecanumDriver extends OpMode {
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pivotPosition = 0;
    private int viperPosition = 0;
    private final static double TURN_POWER = 2.0;
    private final static double FORWARD_POWER = 1.0;
    private final static double VIPER_POWER = 1.0;
    private final static double PIVOT_POWER = 0.7;
    private final static double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final static double SPEED_MULTIPLIER = 2.3;
    public final boolean isFieldCentric = true;
    private Arm arm;
    private Intake intake;

    @Override
    public void init() {
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "BACKLEFT");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "BACKRIGHT");
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "FRONTLEFT");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "FRONTRIGHT");

        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");
        gyro.resetYaw();

        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro);

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
        intake.close();
    }

    @Override
    public void loop() {
//        robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
//                gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
//                gamepad1.right_stick_x * TURN_POWER, isFieldCentric);

        // Viper
        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            viperPosition += (int) ((gamepad1.left_trigger - gamepad1.right_trigger) * VIPER_POWER * 100);
            viperPosition = Math.max(Math.min(Arm.MAX_LIMIT, viperPosition), Arm.MIN_LIMIT);
        } else if (arm.getViperMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm.stopVipers();
        }

        // Pivot
        if (gamepad1.dpad_up) {
            pivotPosition += (int) (60 * PIVOT_POWER);
        } else if (gamepad1.dpad_down) {
            pivotPosition -= (int) (60 * PIVOT_POWER);
        } else if (arm.getPivotMode() != DcMotor.RunMode.RUN_TO_POSITION) {
            arm.stopPivot();
        }

        // Preset Viper Positions
        if (gamepad1.left_bumper) {
            arm.setViperTargetPosition(Arm.MAX_LIMIT, VIPER_POWER);
        } else if (gamepad1.right_bumper) {
            arm.setViperTargetPosition(Arm.MIN_LIMIT, VIPER_POWER);
        }

        // Open/close intake
        if (gamepad1.a) {
            telemetry.addData("Button", "A pressed");
            if (intake.isOpen()) {
                intake.close();
            } else {
                intake.open();
            }
        }

        arm.setPivotTargetPosition(pivotPosition, PIVOT_POWER);
        arm.setViperTargetPosition(viperPosition, VIPER_POWER);

        telemetry.addData("Claw Left Position", intake.getLeftPosition());
        telemetry.addData("Claw Right Position", intake.getRightPosition());
        telemetry.addData("Viper Slide Position", arm.getViperCurrentPosition());
        telemetry.addData("Pivot Position", arm.getPivotCurrentPosition());
        telemetry.addData("", "");
        robotController.sendTelemetry(telemetry);
    }
}
