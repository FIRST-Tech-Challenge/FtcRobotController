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

package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaPinpointDriver;

import java.util.Locale;

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

public class TT_RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;

    public GoBuildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public GoBuildaDriveToPoint nav; //OpMode member for the point-to-point navigation class
    private DcMotor leftLift = null;  //  Used to control the left back drive wheel
    private DcMotor rightLift = null;  //  Used to control the left back drive wheel

    private ScaledServo extensionServo = null;
    private ScaledServo extensionArm = null;
    private boolean extensionArmUp = true;
    private ScaledServo liftArm = null;
    private boolean liftArmUp = true;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public static final double MID_SERVO = 0.5;
    public static final double HAND_SPEED = 0.02;  // sets rate to move servo
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    public int liftHeight = 0;
    public int liftHeightMax = 1500;
    public int liftHeightMin = 0;
    public int liftSafetyThreshold = 50;
    public double effectivePower = 0;
    public double liftPowerMax = .5;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public TT_RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init() {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        leftFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "Left Front");
        leftBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "Left Back");
        rightFrontDrive = myOpMode.hardwareMap.get(DcMotor.class, "Right Front");
        rightBackDrive = myOpMode.hardwareMap.get(DcMotor.class, "Right Back");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftLift = myOpMode.hardwareMap.get(DcMotor.class, "left lift");
        rightLift = myOpMode.hardwareMap.get(DcMotor.class, "right lift");
        leftLift.setDirection(DcMotor.Direction.REVERSE);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftHeight = leftLift.getCurrentPosition();
        rightLift.setTargetPosition(liftHeight);
        leftLift.setTargetPosition(liftHeight);
        rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        odo = myOpMode.hardwareMap.get(GoBuildaPinpointDriver.class, "odo");
        odo.setOffsets(-115.0, -160.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBuildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBuildaPinpointDriver.EncoderDirection.FORWARD, GoBuildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        nav = new GoBuildaDriveToPoint(myOpMode);
        nav.setDriveType(GoBuildaDriveToPoint.DriveType.MECANUM);

        extensionServo = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension"), "Extension",0.0, 1.0);
        extensionArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "ExtensionArm"), "Extension",0.25, 0.5);

        liftArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Arm"), "Arm",0.38, .62);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.addData("X offset", odo.getXOffset());
        myOpMode.telemetry.addData("Y offset", odo.getYOffset());
        myOpMode.telemetry.addData("Device Scalar", odo.getYawScalar());
        myOpMode.telemetry.update();
    }

    public void moveLiftArm() {
        if (myOpMode.gamepad1.x) {
            if (liftArmUp) {liftArm.setTargetPosition(1); }
            else  {liftArm.setTargetPosition(0); }
            liftArmUp = !liftArmUp;
        }
    }

    public void moveExtensionArm() {
        if (myOpMode.gamepad1.y) {
            if (extensionArmUp) {extensionArm.setTargetPosition(1); }
            else  {extensionArm.setTargetPosition(0); }
            extensionArmUp = !extensionArmUp;
        }
    }
    public void drivelift() {
        if (myOpMode.gamepad1.a) {
            raiseLift();
        } else if (myOpMode.gamepad1.b) {
            lowerLift();
        }
    }

    // Lift
    public void raiseLift() {
        liftHeight = liftHeight + 10;
        liftHeight = setLiftPosition(liftHeight);
    }

    public void lowerLift() {
        liftHeight = liftHeight - 20;
        liftHeight = setLiftPosition(liftHeight);
    }

    public int setLiftPosition(int position) {
        double liftSafetyPowerOverride = 0;
        boolean liftSafetyOverride = false;
        int liftSafetyCheck = 0;

        // Height Limits Check
        if (position > liftHeightMax) position = liftHeightMax;
        else if (position < liftHeightMin) position = liftHeightMin;

        // Height Limit approaching - Power Check
        liftSafetyCheck = liftHeightMax - leftLift.getCurrentPosition();

        if (liftSafetyCheck < liftSafetyThreshold) {
            liftSafetyOverride = position > leftLift.getCurrentPosition();
            liftSafetyPowerOverride = (double) liftSafetyCheck / liftSafetyThreshold;
        } else if (leftLift.getCurrentPosition() < liftSafetyThreshold) {
            liftSafetyOverride = position < leftLift.getCurrentPosition();
            liftSafetyPowerOverride = (double) -leftLift.getCurrentPosition() / liftSafetyThreshold;
        } else {
            liftSafetyOverride = false;
        }

        if (Math.abs(liftSafetyPowerOverride) < .15) liftSafetyPowerOverride = .15;

        if (liftSafetyOverride) effectivePower = liftSafetyPowerOverride;
        else effectivePower = liftPowerMax;

        leftLift.setPower(effectivePower);
        rightLift.setPower(effectivePower);
        leftLift.setTargetPosition(position);
        rightLift.setTargetPosition(position);


        return position;

    }

    public void driveRobot() {

        odo.update();
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        //max *= 1.5;

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

    }

    public void setDrivePower(double leftFront, double rightFront, double leftBack, double rightBack) {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(leftFront);
        rightFrontDrive.setPower(rightFront);
        leftBackDrive.setPower(leftBack);
        rightBackDrive.setPower(rightBack);
    }

    public void setDrivePower() {
        // Output the values to the motor drives.
        leftFrontDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_FRONT));
        rightFrontDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_FRONT));
        leftBackDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.LEFT_BACK));
        rightBackDrive.setPower(nav.getMotorPower(GoBuildaDriveToPoint.DriveMotor.RIGHT_BACK));
    }

    public void displayTelemetry() {
        try {
            myOpMode.telemetry.addData("Lift", "Lift Height: %4d",liftHeight);
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Position", data);
            myOpMode.telemetry.addData("Drive Train", "LF %4d LB %4d RF %4d RB %4d", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition());
            myOpMode.telemetry.update();
        } catch (Exception ex) {        }
    }

    /**
     * Pass the requested arm power to the appropriate hardware drive motor
     *
     * @param power driving power (-1.0 to 1.0)
     */
    //public void setArmPower(double power) {
    //  armMotor.setPower(power);
    //}

    /**
     * Send the two hand-servos to opposing (mirrored) positions, based on the passed offset.
     *
     * @param offset
     */
    public void setHandPositions(double offset) {
        offset = Range.clip(offset, -0.5, 0.5);
        //  leftHand.setPosition(MID_SERVO + offset);
        //  rightHand.setPosition(MID_SERVO - offset);
    }
}
