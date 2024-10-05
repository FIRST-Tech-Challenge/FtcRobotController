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

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import static org.firstinspires.ftc.teamcode.Utility.Config.HUB_FACING;
import static org.firstinspires.ftc.teamcode.Utility.Config.TURNING_P_GAIN;

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
    private final Supplier<Boolean> opModeIsActive;
    /* Declare OpMode members. */

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private final DcMotor FLDrive;
    private final DcMotor FRDrive;
    private final DcMotor BLDrive;
    private final DcMotor BRDrive;
    private final IMU imu;
    //private final DcMotor armMotor;
    //private final Servo clawWrist;
    //private final Servo clawFingers;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public final double MID_SERVO = 0.5;
    public final double HAND_SPEED = 0.02;  // sets rate to move servo
    public final double ARM_UP_POWER = 0.45;
    public final double ARM_DOWN_POWER = -0.45;
    // TODO: change this number once we do encoder math for the correct number.
    static final double COUNTS_PER_MOTOR_REV = 1440;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 4.0;
    public final static double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    public final static double ENCODER_PER_INCH = 1.0;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Drivebase(HardwareMap hardwareMap, Supplier<Boolean> opModeIsActive) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        FLDrive = hardwareMap.dcMotor.get("FLDrive");
        FRDrive = hardwareMap.dcMotor.get("FRDrive");
        BLDrive = hardwareMap.dcMotor.get("BLDrive");
        BRDrive = hardwareMap.dcMotor.get("BRDrive");
        //armMotor = hardwareMap.dcMotor.get("arm");

        imu = hardwareMap.get(IMU.class, "IMU");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);
        imu.resetYaw();


        this.opModeIsActive = opModeIsActive;

        FLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        FLDrive.setDirection(DcMotor.Direction.REVERSE);
        FRDrive.setDirection(DcMotor.Direction.FORWARD);
        BLDrive.setDirection(DcMotor.Direction.FORWARD);
        BRDrive.setDirection(DcMotor.Direction.FORWARD);

        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
        //clawWrist = hardwareMap.servo.get("clawWrist");
        //clawFingers = hardwareMap.servo.get("clawFingers");
        //clawWrist.setPosition(MID_SERVO);
        //clawFingers.setPosition(MID_SERVO);
    }

    /**
     * For use in teleop, controlled by a controller.
     *
     * @param yInput    Driving input. Negative is back, positive is forward. [-1, 1].
     * @param xInput    Strafing input. Negative is left, positive is right. [-1, 1].
     * @param turnInput Turning input. Negative is ccw, positive is clockwise. [-1, 1].
     */
    public void driveRobot(double yInput, double xInput, double turnInput) {
        double frontLeft = yInput + xInput + turnInput;
        double frontRight = yInput - xInput - turnInput;
        double backLeft = yInput - xInput + turnInput;
        double backRight = yInput + xInput - turnInput;

        double clamp = maxOf(1, frontLeft, frontRight, backLeft, backRight);

        setDrivePowers(
                frontLeft / clamp,
                frontRight / clamp,
                backLeft / clamp,
                backRight / clamp
        );
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void autoDriveForward(double speed, double inchesForward) {
        int newFLTarget = 0;
        int newFRTarget = 0;
        int newBLTarget = 0;
        int newBRTarget = 0;

        if (opModeIsActive.get()) {
            //Resetting encoders
            FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            newFLTarget = FLDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newFRTarget = FRDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newBLTarget = BLDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            newBRTarget = BRDrive.getCurrentPosition() + (int) (inchesForward * COUNTS_PER_INCH);
            FLDrive.setTargetPosition(newFLTarget);
            FRDrive.setTargetPosition(newFRTarget);
            BLDrive.setTargetPosition(newBLTarget);
            BRDrive.setTargetPosition(newBRTarget);

            // Determine new target position, and pass to motor controller

            // Turn On RUN_TO_POSITION
            FLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BLDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BRDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FLDrive.setPower(Math.abs(speed));
            FRDrive.setPower(Math.abs(speed));
            BLDrive.setPower(Math.abs(speed));
            BRDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive.get() && (FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy()) && telemetry != null) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newFLTarget, newFRTarget);
                telemetry.addData("Currently at", " at %7d :%7d", FLDrive.getCurrentPosition(), FRDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            FLDrive.setPower(0);
            FRDrive.setPower(0);
            BLDrive.setPower(0);
            BRDrive.setPower(0);

            //Turn off RUN_TO_POSITION
           FLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           FRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           BLDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           BRDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void driveSideways(double speed, double inches) {
        //Make it to where encoders reset.
        FLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BLDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BRDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double target = inches * ENCODER_PER_INCH;
        setMotorTargets((int) target, (int) -target, (int) -target, (int) target);

        setMotorModes(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePowers(speed);
        waitForMotors(telemetry);

        setDrivePowers(0);
    }

    /**
     * For auto. Turns to a specified angle, without resetting the heading.
     *
     * @param angle     How many degrees to turn. [-180, 180].
     * @param speed     How fast to turn. (0, 1].
     * @param telemetry Pass this if you want this method to log to telemetry.
     * @see Drivebase#driveSideways
     * @see Drivebase#autoDriveForward
     * @see Drivebase#turnToAngle
     */
    public void turnToAngle(double speed, double angle, @Nullable Telemetry telemetry) {
        setMotorModes(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive.get() && Math.abs(getHeading() - angle) > 1) {
            final double adjustedPower = speed * getTurningCorrection(angle);

            setDrivePowers(adjustedPower, -adjustedPower, adjustedPower, -adjustedPower);

            if (telemetry == null) continue; else addTelemetry(telemetry);

        }

        setDrivePowers(0);
    }

    /**
     * Ensures that an angle is within [-180, 180].
     *
     * @param n Angle in degrees.
     * @return Angle wrapped into [-180, 180].
     */
    private static double wrapAngle(double n) {
        double x = (n % 360 + 360) % 360;
        return x < 180 ? x : x - 360;
    }

    private double getTurningCorrection(double angle) {
        return Range.clip(wrapAngle(angle - getHeading()) * TURNING_P_GAIN, -1, 1);
    }

    /**
     * Pass the requested wheel motor powers to the appropriate hardware drive motors.
     *
     * @param FLDrivePower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param FRDrivePower Fwd/Rev driving power (-1.0 to 1.0) +ve is forward
     * @param BLDrivePower
     * @param BRDrivePower
     */
    public void setDrivePowers(double FLDrivePower, double FRDrivePower, double BLDrivePower, double BRDrivePower) {
        // Output the values to the motor drives.
        FLDrive.setPower(FLDrivePower);
        FRDrive.setPower(FRDrivePower);
        BLDrive.setPower(BLDrivePower);
        BRDrive.setPower(BRDrivePower);
    }

    public void setDrivePowers(double power) {
        // Output the values to the motor drives.
        FLDrive.setPower(power);
        FRDrive.setPower(power);
        BLDrive.setPower(power);
        BRDrive.setPower(power);
    }

    /**
     * Waits until the motors are finished moving, or the driver presses stop.
     *
     * @param telemetry Pass this if you want to log to telemetry.
     */
    private void waitForMotors(@Nullable Telemetry telemetry) {
        while (opModeIsActive.get() && (FLDrive.isBusy() || FRDrive.isBusy() || BLDrive.isBusy() || BRDrive.isBusy())) {
            if (telemetry == null) continue; else addTelemetry(telemetry);
        }
    }

    private void setMotorTargets(int fltarget, int frtarget, int bltarget, int brtarget) {
        FLDrive.setTargetPosition(fltarget);
        FRDrive.setTargetPosition(frtarget);
        BLDrive.setTargetPosition(bltarget);
        BRDrive.setTargetPosition(brtarget);
    }

    private void setMotorModes(DcMotor.RunMode mode) {
        FLDrive.setMode(mode);
        FRDrive.setMode(mode);
        BLDrive.setMode(mode);
        BRDrive.setMode(mode);
    }

    /**
     * @return The robot heading in degrees.
     */
    private double getHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * @param doubles Numbers to look over.
     * @return The maximum number.
     */
    private double maxOf(double... doubles) {
        double max = -Double.MAX_VALUE;
        for (double x : doubles) if (x > max) max = x;
        return max;
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    //public void liftArm(double power) {
        //armMotor.setPower(power);
    //}

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void liftClaw(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        //clawWrist.setPosition(MID_SERVO + offset);
    }

    public void moveFingers(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        //clawFingers.setPosition(MID_SERVO - offset);
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
