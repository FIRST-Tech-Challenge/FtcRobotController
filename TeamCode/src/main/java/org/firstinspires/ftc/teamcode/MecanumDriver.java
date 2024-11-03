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

import android.view.ViewOutlineProvider;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;


@TeleOp(name="Mecanum Driver", group="TeleOp")
public class MecanumDriver extends OpMode {
    private MecanumRobotController robotController;
    private final ElapsedTime runtime = new ElapsedTime();
    private int pivotPosition = 0;
    private int viperPosition = 0;
    private double timeAPressed = 0;
    private double timeXPressed = 0;
    private final static double TURN_POWER = 2.0;
    private final static double FORWARD_POWER = 1.0;
    private final static double VIPER_VELOCITY_CONSTANT = 1800;
    private final static double BASE_PIVOT_VELOCITY = 240;
    private final static double MAX_PIVOT_VELOCITY = 840;
    private final static double PIVOT_RAMP_TIME = 1.0;
    private final static double STRAFE_POWER = FORWARD_POWER * 1.192;
    private final static double SPEED_MULTIPLIER = 2.3;
    private final static double INTAKE_COOLDOWN = 0.25;
    private final static double WHACK_COOLDOWN = 0.25;
    private final static double WRIST_MIN_POS = 0.0;
    private final static double WRIST_MAX_POS = 0.5;
    public static final double MAX_EXTENSION_BACK = 7.0;
    public static final double MAX_EXTENSION_FORWARD = 17.0;
    public final boolean isFieldCentric = true;
//    private Servo claw;
//    private Servo wrist;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    private double pivotStartedTime;
    private boolean pivotStarted;
//    private boolean clawOpen;

    @Override
    public void init() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
//        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
//        claw.setDirection(Servo.Direction.REVERSE);
//        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");

        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        IMU gyro = hardwareMap.get(IMU.class, "imu2");

        viperSlide = new ViperSlide(
                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
        );
        pivot = new Pivot(
                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT")
        );
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
        intake.unwhack();
//        claw.setPosition(0.75);
//        wrist.setPosition(WRIST_MIN_POS);
    }

    @Override
    public void loop() {
        // Drivetrain
        robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
                gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
                gamepad1.right_stick_x * TURN_POWER, isFieldCentric);

        double pivotPower = Math.min(MAX_PIVOT_VELOCITY, BASE_PIVOT_VELOCITY + (MAX_PIVOT_VELOCITY - BASE_PIVOT_VELOCITY) * (runtime.seconds() - pivotStartedTime) / PIVOT_RAMP_TIME);
        // Pivot
        if (gamepad2.dpad_down) {
            if (!pivotStarted) {
                pivotStartedTime = runtime.seconds();
                pivotStarted = true;
            }
            pivot.move(pivotPower);
        } else if (gamepad2.dpad_up) {
            if (!pivotStarted) {
                pivotStartedTime = runtime.seconds();
                pivotStarted = true;
            }
            pivot.move(-pivotPower);
        } else {
            pivot.hold();
            pivotStarted = false;
        }

        double triggerPower = gamepad2.left_trigger - gamepad2.right_trigger;
        double extensionBeyondChassis = viperSlide.getExtensionBeyondChassis(pivot.getAngleDegrees());
        // Viper
        if (triggerPower < 0 || (triggerPower != 0 && extensionBeyondChassis < MAX_EXTENSION_FORWARD && extensionBeyondChassis > -MAX_EXTENSION_BACK)) {
            viperSlide.move(triggerPower * VIPER_VELOCITY_CONSTANT);
        } else {
            viperSlide.hold();
        }

        if (extensionBeyondChassis > MAX_EXTENSION_FORWARD) {
            viperSlide.setTargetPosition((int) (1 / Math.cos(pivot.getAngleDegrees()) * (MAX_EXTENSION_FORWARD + ViperSlide.CHASSIS_TO_PIVOT_LENGTH) * ViperSlide.MOVE_COUNTS_PER_INCH));
        } else if (-extensionBeyondChassis < MAX_EXTENSION_BACK) {
            viperSlide.setTargetPosition((int) Math.abs((1 / Math.cos(pivot.getAngleDegrees()) * (MAX_EXTENSION_BACK + ViperSlide.CHASSIS_TO_PIVOT_LENGTH) * ViperSlide.MOVE_COUNTS_PER_INCH)));
        }

        // Preset Viper Positions
        if (gamepad2.left_bumper) {
            viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
        } else if (gamepad2.right_bumper) {
            viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
        }

        // Open/close intake
        if (gamepad2.a && runtime.seconds() - timeAPressed >= INTAKE_COOLDOWN) {
            telemetry.addData("Button", "A pressed");
            timeAPressed = runtime.seconds();
            if (intake.isOpen()/*clawOpen*/) {
                intake.close();
//                claw.setPosition(0.75);
//                clawOpen = false;
            } else {
                intake.open();
//                claw.setPosition(0.65);
//                clawOpen = true;
            }
        }

        if (gamepad2.x && runtime.seconds() - timeXPressed >= WHACK_COOLDOWN) {
            timeXPressed = runtime.seconds();
            if (intake.isWhacked()) {
                intake.unwhack();
            } else {
                intake.whack();
            }
        }

//        if (gamepad2.dpad_left) {
//            wrist.setPosition(Math.max(WRIST_MIN_POS, wrist.getPosition() - 0.03));
//        } else if (gamepad2.dpad_right) {
//            wrist.setPosition(Math.min(WRIST_MAX_POS, wrist.getPosition() + 0.03));
//        }

        telemetry.addData("Claw Left Position", intake.getLeftPosition());
        telemetry.addData("Claw Right Position", intake.getRightPosition());
//        telemetry.addData("Claw Position", claw.getPosition());
//        telemetry.addData("Wrist Position", wrist.getPosition());
        telemetry.addData("Viper Slide Position", viperSlide.getCurrentPosition());
        telemetry.addData("Viper Extension Beyond Chassis", extensionBeyondChassis);
        telemetry.addData("Viper Left Pos", viperSlide.getLeftPosition());
        telemetry.addData("Viper Right Pos", viperSlide.getRightPosition());
        telemetry.addData("Pivot Position", pivot.getCurrentPosition());
        telemetry.addData("Pivot Angle", pivot.getAngleDegrees());
        telemetry.addData("Whacker Position", intake.getWhackerPosition());
        telemetry.addData("", "");
        robotController.sendTelemetry(telemetry);
    }
}
