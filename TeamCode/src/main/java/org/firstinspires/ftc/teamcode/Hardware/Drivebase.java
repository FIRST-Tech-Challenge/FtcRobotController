/* Copyright (c) 2022 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.Hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or... In OnBot Java, add a new file named RobotHardware.java, select this sample, and select Not an OpMode.
 * Also add a new OpMode, select the sample ConceptExternalHardwareClass.java, and select TeleOp.
 *
 */

public class Drivebase {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    static private DcMotor FLDrive = null;
    static private DcMotor FRDrive = null;
    static private DcMotor BLDrive = null;
    static private DcMotor BRDrive = null;
    static private DcMotor armMotor = null;
    static private Servo   leftHand = null;
    static private Servo   rightHand = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO       =  0.5 ;
    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Drivebase (HardwareMap hardwareMap) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLDrive = myOpMode.hardwareMap.get(DcMotor.class, "FLDrive");
        FRDrive = myOpMode.hardwareMap.get(DcMotor.class, "FRDrive");
        BLDrive = myOpMode.hardwareMap.get(DcMotor.class, "BLDrive");
        BRDrive = myOpMode.hardwareMap.get(DcMotor.class, "BRDrive");
        armMotor = myOpMode.hardwareMap.get(DcMotor.class, "arm");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.REVERSE);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        leftHand = myOpMode.hardwareMap.get(Servo.class, "left_hand");
        rightHand = myOpMode.hardwareMap.get(Servo.class, "right_hand");
        leftHand.setPosition(MID_SERVO);
        rightHand.setPosition(MID_SERVO);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Calculates the left/right motor powers required to achieve the requested
     * robot motions: Drive (Axial motion) and Turn (Yaw motion).
     * Then sends these power levels to the motors.
     *
     * @param axial     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param lateral   FWD/Rev driving power (-1.0 to 1.0) +ve is right
     * @param yaw      Right/Left turning power (-1.0 to 1.0) +ve is CW
     */
    public void driveRobot(double axial, double lateral, double yaw) {
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double FLDrivePower  = axial + lateral + yaw;
        double FRDrivePower = axial - lateral - yaw;
        double BLDrivePower   = axial - lateral + yaw;
        double BRDrivePower  = axial + lateral - yaw;

        // Scale the values so neither exceed +/- 1.0
        double frontMax = Math.max(Math.abs(FLDrivePower), Math.abs(FRDrivePower));
        double backMax = Math.max(Math.abs(BLDrivePower), Math.abs(BRDrivePower));

        if (frontMax > 1.0 && backMax > 1.0)
        {
            FLDrivePower /= frontMax;
            FRDrivePower /= frontMax;
            BLDrivePower /= backMax;
            BRDrivePower /= backMax;
        }

        // Use existing function to drive all wheels.
        setDrivePower(FLDrivePower, FRDrivePower, BLDrivePower, BRDrivePower);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param FLDrivePower     Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param FRDrivePower    Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param BLDrivePower
     * @param BRDrivePower
     */
    public void setDrivePower(double FLDrivePower, double FRDrivePower, double BLDrivePower, double BRDrivePower) {
        // Output the values to the motor drives.
        FLDrive.setPower(FLDrivePower);
        FRDrive.setPower(FRDrivePower);
        BLDrive.setPower(BLDrivePower);
        BRDrive.setPower(BRDrivePower);
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    public void setArmPower(double power) {
        armMotor.setPower(power);
    }

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    static public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        leftHand.setPosition(MID_SERVO + offset);
        rightHand.setPosition(MID_SERVO - offset);
    }

    public void addTelemetry(Telemetry telemetry) {
        // Show the wheel powers.
        telemetry.addData("FLDrive", "%4.2f, %4.2f", FLDrive.getCurrentPosition());
        telemetry.addData("FRDrive", "%4.2f, %4.2f", FRDrive.getCurrentPosition());
        telemetry.addData("BLDrive", "%4.2f, %4.2f", BLDrive.getCurrentPosition());
        telemetry.addData("BRDrive", "%4.2f, %4.2f", BRDrive.getCurrentPosition());
        telemetry.update();
    }
}
