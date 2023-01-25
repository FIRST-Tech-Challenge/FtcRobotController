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

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvWebcam;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for the robot.
 * <p>
 */
@Config
public class HuskyBot {
    /* Public OpMode members. */
    public SampleMecanumDrive drive = null;

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
    public Servo clawGrab = null;

    // Webcam
    public OpenCvWebcam webcam;

    // Magnetic Limit Switches
    public TouchSensor armExtendMax = null;
    public TouchSensor armExtendMin = null;

    // goBILDA 5203 Series Yellow Jacket Planetary Gear Motor
    // max encoder ticks per second
    // https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/
    public static final double VELOCITY_CONSTANT = 537.7 * 312/60;


    // Define Arm Constants: Power
    public static final double ARM_SWIVEL_MAX_POWER = 0.35;
    public static final double ARM_LIFT_MAX_POWER = 0.5;
    public static final double ARM_LIFT_MIN_POWER = 0.01;
    public static final double ARM_LIFT_POWER_AT_REST = 0.12;
    public static final double ARM_EXTENSION_MAX_POWER = 0.6;

    // Define Arm Constants: Encoder and Servo Values
    public static final int ARM_SWIVEL_RIGHT_LIMIT = -70;
    public static final int ARM_SWIVEL_LEFT_LIMIT = 160;

    public static final int ARM_LIFT_MAX_POSITION = 910;

    public static final double CLAW_MOVE_INCREMENT = 0.05;
    public static final double CLAW_LIFT_MIN_RANGE = 0.0;
    public static final double CLAW_LIFT_MAX_RANGE = 1.0;
    public static final double CLAW_LIFT_START_POSITION = 0.6;   // scaled, see MIN and MAX_RANGE

    public static final double CLAW_GRAB_MIN_RANGE = 0.1;
    public static final double CLAW_GRAB_MAX_RANGE = 0.54;
    public static final double CLAW_GRAB_OPEN_POSITION = 0.3;
    public static final double CLAW_GRAB_CLOSE_POSITION = 1.0;

    // Define Arm Constants: Preset Junction Positions
    public static final int ARM_LIFT_GROUND_POSITION = 20, ARM_EXTEND_GROUND_POSITION = -1825; public static final double CLAW_LIFT_GROUND_POSITION = 0.55;
    public static final int ARM_LIFT_LOW_POSITION = 360, ARM_EXTEND_LOW_POSITION = -10; public static final double CLAW_LIFT_LOW_POSITION = 0.55;
    public static final int ARM_LIFT_MED_POSITION = 660, ARM_EXTEND_MED_POSITION = -10; public static final double CLAW_LIFT_MED_POSITION = 0.50;
    public static final int ARM_LIFT_HIGH_POSITION = 880, ARM_EXTEND_HIGH_POSITION = -3215; public static final double CLAW_LIFT_HIGH_POSITION = 0.35;

    /* local OpMode members. */
    HardwareMap hwMap = null;

    public HuskyBot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Mecanum Drive
        drive = new SampleMecanumDrive(ahwMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and Initialize Drive Motors
//        frontLeftDrive = hwMap.get(DcMotorEx.class, "front_left_drive");
//        rearLeftDrive = hwMap.get(DcMotorEx.class, "rear_left_drive");
//        frontRightDrive = hwMap.get(DcMotorEx.class, "front_right_drive");
//        rearRightDrive = hwMap.get(DcMotorEx.class, "rear_right_drive");
//
//        // Set Drive Motors to Zero Power
//        frontLeftDrive.setPower(0);
//        rearLeftDrive.setPower(0);
//        frontRightDrive.setPower(0);
//        rearRightDrive.setPower(0);
//
//        // Reset Drive Motor Encoders
//        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Set Drive Motor Behaviors
//        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rearRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rearLeftDrive.setDirection(DcMotor.Direction.REVERSE);
//
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set Drive Motor PIDF Coefficients
        // https://docs.google.com/document/u/1/d/1tyWrXDfMidwYyP_5H4mZyVgaEswhOC35gvdmP-V-5hA/mobilebasic
        // todo these still need to be tuned
//        frontLeftDrive.setVelocityPIDFCoefficients(1.82, 0.182, 0, 18.2);
//        frontLeftDrive.setPositionPIDFCoefficients(5.0);
//        rearLeftDrive.setVelocityPIDFCoefficients(1.18, 0.118, 0, 11.8);
//        rearLeftDrive.setPositionPIDFCoefficients(5.0);
//        frontRightDrive.setVelocityPIDFCoefficients(1.43, 0.143, 0, 14.3);
//        frontRightDrive.setPositionPIDFCoefficients(5.0);
//        rearRightDrive.setVelocityPIDFCoefficients(1.27, 0.127, 0, 12.7);
//        rearRightDrive.setPositionPIDFCoefficients(5.0);


        // Define and Initialize Arm Motors and Servos
        armSwivelMotor = hwMap.get(DcMotorEx.class, "arm_swivel");
        armLiftMotor = hwMap.get(DcMotorEx.class, "arm_lift");
        armExtendMotor = hwMap.get(DcMotorEx.class, "arm_extend");

        clawLift = hwMap.get(Servo.class, "claw_lift");
        clawGrab = hwMap.get(Servo.class, "claw_grab");
        clawLift.scaleRange(CLAW_LIFT_MIN_RANGE, CLAW_LIFT_MAX_RANGE);
        clawGrab.scaleRange(CLAW_GRAB_MIN_RANGE, CLAW_GRAB_MAX_RANGE);

        // Define and Init. Magnetic Limit Switches
        armExtendMax = hwMap.get(TouchSensor.class, "arm_extend_max");
        armExtendMin = hwMap.get(TouchSensor.class, "arm_extend_min");

        // Set Arm Motors to Zero Power
        armSwivelMotor.setPower(0);
        armLiftMotor.setPower(0);
        armExtendMotor.setPower(0);

        // Reset Arm Motor Encoders
        armSwivelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Arm Motor Behaviors
        armSwivelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armLiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armExtendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        armSwivelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armExtendMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
