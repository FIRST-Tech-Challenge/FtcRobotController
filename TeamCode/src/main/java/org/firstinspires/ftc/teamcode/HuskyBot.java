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

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for the robot.
 * <p>
 */
public class HuskyBot {
    /* Public OpMode members. */
    public DcMotorEx frontLeftDrive = null;
    public DcMotorEx frontRightDrive = null;
    public DcMotorEx rearLeftDrive = null;
    public DcMotorEx rearRightDrive = null;

    // Arm Control Motor Init.
    public DcMotorEx armSwivelMotor = null;
    public DcMotorEx armLiftMotor = null;
    public DcMotorEx armExtendMotor = null;

    // Claw (on the Arm) Servo Init.
    public Servo clawLift = null;
    public Servo clawRotate = null;
    public Servo clawGrab = null; // TODO: set this to be fixed open/close positions.

    // goBILDA 5203 Series Yellow Jacket Planetary Gear Motor
    // max encoder ticks per second
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double VELOCITY_CONSTANT = 537.7 * 312/60;


    public static final double ARM_SWIVEL_MAX_POWER = 0.4;
    public static final double ARM_SWIVEL_LIMIT = 200;
    public static final double ARM_LIFT_MAX_POWER = 0.25;
    public static final double ARM_LIFT_MIN_POWER = 0.01;
    public static final double ARM_LIFT_POWER_AT_REST = 0.05;
    public static final double ARM_EXTENSION_MAX_POWER = 0.4;

    public static final double CLAW_MOVE_INCREMENT = 0.01;

    public static final double CLAW_LIFT_MIN_RANGE = 0.3;
    public static final double CLAW_LIFT_MAX_RANGE = 0.8;
    public static final double CLAW_LIFT_START_POSITION = 1.0;   // scaled, see MIN and MAX_RANGE

    public static final double CLAW_ROTATE_MIN_RANGE = 0.1;
    public static final double CLAW_ROTATE_MAX_RANGE = 0.8;
    public static final double CLAW_ROTATE_START_POSITION = 1.0;   // scaled, see MIN and MAX_RANGE

    public static final double CLAW_GRAB_MIN_RANGE = 0.1;
    public static final double CLAW_GRAB_MAX_RANGE = 0.54;
    public static final double CLAW_GRAB_OPEN_POSITION = 0.3;
    public static final double CLAW_GRAB_CLOSE_POSITION = 1.0;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    public HuskyBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeftDrive = hwMap.get(DcMotorEx.class, "front_left_drive");
        rearLeftDrive = hwMap.get(DcMotorEx.class, "rear_left_drive");
        frontRightDrive = hwMap.get(DcMotorEx.class, "front_right_drive");
        rearRightDrive = hwMap.get(DcMotorEx.class, "rear_right_drive");

        // Define and Init. Arm Motors
        armSwivelMotor = hwMap.get(DcMotorEx.class, "arm_swivel");
        armLiftMotor = hwMap.get(DcMotorEx.class, "arm_lift");
        armExtendMotor = hwMap.get(DcMotorEx.class, "arm_extend");

        // Define and Init. Claw Servos
        clawRotate = hwMap.get(Servo.class, "claw_rotate");
        clawLift = hwMap.get(Servo.class, "claw_lift");
        clawGrab = hwMap.get(Servo.class, "claw_grab");

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        frontLeftDrive.setPower(0);
        rearLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        rearRightDrive.setPower(0);

        // Set all arm-related motors and servos to zero power.
        armSwivelMotor.setPower(0);

        armLiftMotor.setPower(0);
        armLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armExtendMotor.setPower(0);

        clawRotate.scaleRange(CLAW_ROTATE_MIN_RANGE, CLAW_ROTATE_MAX_RANGE);
        clawLift.scaleRange(CLAW_LIFT_MIN_RANGE, CLAW_LIFT_MAX_RANGE);
        clawGrab.scaleRange(CLAW_GRAB_MIN_RANGE, CLAW_GRAB_MAX_RANGE);

        // this base configuration sets the drive motors to run without encoders and the arm motor
        // to run with encoder. if any opmode requires different setting, that should be changed in
        // the opmode itself
        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // https://docs.google.com/document/u/1/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
        // todo these still need to be tuned
        frontLeftDrive.setVelocityPIDFCoefficients(1.82, 0.182, 0, 18.2);
        frontLeftDrive.setPositionPIDFCoefficients(5.0);
        rearLeftDrive.setVelocityPIDFCoefficients(1.18, 0.118, 0, 11.8);
        rearLeftDrive.setPositionPIDFCoefficients(5.0);
        frontRightDrive.setVelocityPIDFCoefficients(1.43, 0.143, 0, 14.3);
        frontRightDrive.setPositionPIDFCoefficients(5.0);
        rearRightDrive.setVelocityPIDFCoefficients(1.27, 0.127, 0, 12.7);
        rearRightDrive.setPositionPIDFCoefficients(5.0);
    }

    public void servoMove(Servo servo, double targetPosition) {
        double currentPosition = servo.getPosition();
        if (targetPosition > 0) {
            servo.setPosition(currentPosition + CLAW_MOVE_INCREMENT);
        }
        else if (targetPosition < 0) {
            servo.setPosition(currentPosition - CLAW_MOVE_INCREMENT);
        }
    }
}
