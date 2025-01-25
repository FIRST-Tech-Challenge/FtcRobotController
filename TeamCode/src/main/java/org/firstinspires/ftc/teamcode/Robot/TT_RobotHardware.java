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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaDriveToPoint;
import org.firstinspires.ftc.teamcode.GoBuilda.GoBuildaPinpointDriver;

import java.util.Locale;

public class TT_RobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor leftFrontDrive = null;
    public DcMotor leftBackDrive = null;
    public DcMotor rightFrontDrive = null;
    public DcMotor rightBackDrive = null;
    public DcMotor leftLift = null;  //  Used to control the left back drive wheel
    public DcMotor rightLift = null;  //  Used to control the left back drive wheel

    public GoBuildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    public GoBuildaDriveToPoint nav; //OpMode member for the point-to-point navigation class
    public Pose2D targetPosition;
    public double MaxPowerAdjustment = 1;

    public DigitalChannel liftSafetyButton = null;
    public ScaledServo extension = null;
    public ScaledServo extensionArm = null;
    public double EXT_ArmPickupReadyHeight = 0.6;
    public double EXT_ArmPickupHeight = 1;
    public double EXT_ArmDropHeight = 0;
    public double EXT_ArmMidHeight = 0.3;

    public double extArmPickupReadyHeight = 0.6;
    public ScaledServo extensionGripper = null;
    public ScaledServo extensionSpin = null;
    public ScaledServo liftArm = null;
    public ScaledServo light = null;

    // Initialize servo settings
    public boolean extensionArmUp = true;
    public boolean extensionGripperOpen = true;
    private boolean extensionArmButtonPress = false;
    private boolean lift = true;
    private boolean liftArmUp = true;
    private boolean liftArmButtonPress = false;

    private boolean pickupGestureInitiated = false;
    private ElapsedTime pickupGestureTimer = new ElapsedTime();
    private boolean dropSampleGestureInitatied = false;
    private ElapsedTime dropSampleGestureTimer = new ElapsedTime();

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    public  final double LIFT_ARM_UP = 0;
    public final double LIFT_ARM_DOWN = 1;  // sets rate to move servo
    public final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    private boolean liftEnabled = false;
    public int liftHeight = 0;
    public int liftHeightSpecimenDrop = 1250;
    public int liftHeightMax = 1650;

    public int liftHeightMin = 0;
    public int liftSafetyThreshold = 50;
    public int liftOffset = 0;
    public double effectivePower = 0;
    public double liftPowerMax = 1;
    public double LIGHT_RED = 0.279;
    public double LIGHT_GREEN = 0.5;

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

        liftSafetyButton = myOpMode.hardwareMap.get(DigitalChannel.class, "Lift Safety Button");
        liftSafetyButton.setMode(DigitalChannel.Mode.INPUT);
        if (!liftSafetyButton.getState()) {
            liftEnabled = true;
        }

        odo = myOpMode.hardwareMap.get(GoBuildaPinpointDriver.class, "odo");
        odo.setOffsets(-145.0, -200.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBuildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBuildaPinpointDriver.EncoderDirection.FORWARD, GoBuildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        nav = new GoBuildaDriveToPoint(myOpMode);

        extension = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension"), "Extension", 0.0, 0.6);
        extension.setTargetPosition(1.0);
        extensionSpin = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Spin"), "Extension Spin", .85, 0.95);
        extensionSpin.setTargetPosition(0.5);
        // Extension Arm - higher value is extended.
        extensionArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Arm"), "Extension Arm", 0.73, 1);
        extensionArm.setTargetPosition(EXT_ArmMidHeight);
        extensionGripper = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Extension Gripper"), "Extension Gripper", 0.55, .85);
        extensionGripper.setTargetPosition(1);
        light = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Light"), "Light", 0.0, 1.0);
        // Lift Arm - lower is high height
        liftArm = new ScaledServo(myOpMode.hardwareMap.get(Servo.class, "Lift Arm"), "Lift Arm", 0.4, 1);
        liftArm.setTargetPosition(LIFT_ARM_DOWN);


        displayTelemetry();
    }

    public void checkLiftPosition() {
        if (liftSafetyButton.getState()) {
            light.setTargetPosition(LIGHT_RED);
        } else {
            light.setTargetPosition(LIGHT_GREEN);
        }
    }

    public void savePosition() {
        if (myOpMode.gamepad1.left_stick_button) {
            targetPosition = odo.getPosition();
        }
    }

    public void moveLiftArm() {
        if (myOpMode.gamepad1.right_stick_button) {
            if (!liftArmButtonPress) {
                if (liftArmUp) {
                    liftArm.setTargetPosition(LIFT_ARM_DOWN);
                } else {
                    liftArm.setTargetPosition(LIFT_ARM_UP);
                }
                liftArmUp = !liftArmUp;
            }
            liftArmButtonPress = true;
        } else {
            liftArmButtonPress = false;
        }
    }

    public void moveExtensionArm() {
        if (myOpMode.gamepad1.b) {
            // positioned over blocks for easy viewing
            extensionArm.setTargetPosition(EXT_ArmPickupReadyHeight);
            extensionGripper.setTargetPosition(.5);
        } else if (myOpMode.gamepad1.a) {
            // pickup the block
            if (pickupGestureInitiated == false) {
                pickupGestureInitiated = true;
                pickupGestureTimer.reset();
                extensionArm.setTargetPosition(EXT_ArmPickupHeight);
            }
        } else if (myOpMode.gamepad1.x) {
            // bring arm back for returning block.
            //transferGestureInitiated = true;
            //transferGestureTimer.reset();
            extensionArm.setTargetPosition(EXT_ArmDropHeight);
            extensionSpin.setTargetPosition(.5);
            extension.setTargetPosition(.7);
        } else if (myOpMode.gamepad1.y) {

            // Open Gripper - Drop block into Transfer box
            extensionGripper.setTargetPosition(.5);
            dropSampleGestureInitatied = true;
            dropSampleGestureTimer.reset();


        }
        if (pickupGestureInitiated) {
            if (pickupGestureTimer.seconds() > .2) {
                extensionGripper.setTargetPosition(0);
            }
            if (pickupGestureTimer.seconds() > .3) {
                extensionArm.setTargetPosition(0.6);
                pickupGestureInitiated = false;
            }
        } else if (dropSampleGestureInitatied) {
            if (dropSampleGestureTimer.milliseconds() > 300) {
                // Lower arm to prepare for pickup and bring extension in
                // raise lift in preparation for dropping into scoring box
                extensionArm.setTargetPosition(EXT_ArmPickupReadyHeight);
                extension.setTargetPosition(1);
                setLiftPosition(liftHeightMax);
            }
            if (dropSampleGestureTimer.milliseconds() > 2500) {
                liftArm.setTargetPosition(LIFT_ARM_UP);
            }
            if (dropSampleGestureTimer.milliseconds() > 2750) {
                liftArm.setTargetPosition(LIFT_ARM_DOWN);
            }
            if (dropSampleGestureTimer.milliseconds() > 3000) {
                liftPowerMax = 0.8;
                setLiftPosition(liftHeightMin);
                extensionArm.setTargetPosition(EXT_ArmPickupReadyHeight);
                extensionGripper.setTargetPosition(.5);
                dropSampleGestureInitatied = false;
            }
        }
    }

    public void moveExtension() {

        if (myOpMode.gamepad1.right_trigger > 0) {
            extension.setTargetPosition(extension.getPosition() - (myOpMode.gamepad1.right_trigger / 30));
        } else if (myOpMode.gamepad1.left_trigger > 0) {
            extension.setTargetPosition(extension.getPosition() + (myOpMode.gamepad1.left_trigger / 30));
        } else {
            extension.setTargetPosition(extension.getPosition());
        }
        if (extension.getTargetPosition() > .7 && extensionArm.getTargetPosition() < 0.4) {
            extensionArm.setTargetPosition(Math.abs(.7 - extension.getTargetPosition()));
        }
    }

    public void moveExtensionSpin() {
        double increment = .05;
        double currentPosition = extensionSpin.getPosition();
        if (myOpMode.gamepad1.left_bumper) {
            extensionSpin.setTargetPosition(currentPosition - increment);
        } else if (myOpMode.gamepad1.right_bumper) {
            extensionSpin.setTargetPosition(currentPosition + increment);
        }
    }

    public void drivelift() {
        if (liftEnabled || !liftEnabled) {
            if (myOpMode.gamepad1.dpad_down) {
                setLiftPosition(liftHeightMin);
            } else if (myOpMode.gamepad1.dpad_up) {
                setLiftPosition(liftHeightMax);
            } else if (myOpMode.gamepad1.dpad_right) {
                setLiftPosition(liftHeightSpecimenDrop);
            }
        }
    }

    // Lift
    public void raiseLift() {
        liftHeight = liftHeight + 20;
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
        if (liftSafetyButton.getState() == false) {
            int liftHeightChange = 0;
            liftHeightChange = leftLift.getCurrentPosition() - liftHeightMin;
            liftHeightMin = leftLift.getCurrentPosition();
            liftHeightMax = liftHeightMax + liftHeightChange;
            liftHeightSpecimenDrop = liftHeightSpecimenDrop + liftHeightChange;
            if (leftLift.getCurrentPosition() > rightLift.getCurrentPosition()) {
                liftOffset = leftLift.getCurrentPosition() - rightLift.getCurrentPosition();
            } else {
                liftOffset = leftLift.getCurrentPosition() - rightLift.getCurrentPosition();
            }
        }

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

        if (Math.abs(liftSafetyPowerOverride) < .25) liftSafetyPowerOverride = .25;

        if (liftSafetyOverride) effectivePower = liftSafetyPowerOverride;
        else effectivePower = liftPowerMax;

        leftLift.setPower(effectivePower);
        rightLift.setPower(effectivePower);
        leftLift.setTargetPosition(position);
        rightLift.setTargetPosition(position ); //- liftOffset

        // Save battery power, but not pulling down when not needed.
        if (leftLift.getCurrentPosition() < (liftHeightMin + 25) && leftLift.getTargetPosition() < (liftHeightMin + 25)) {
            leftLift.setPower(0);
            rightLift.setPower(0);
        }
        return position;
    }

    public void driveRobot() {

        odo.update();
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        /*double axial = -myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = myOpMode.gamepad1.left_stick_x;
        double yaw = myOpMode.gamepad1.right_stick_x;
         */
        double axial = myOpMode.gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = -myOpMode.gamepad1.left_stick_x;
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

        max *= MaxPowerAdjustment;

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        setDrivePower(leftFrontPower, rightFrontPower, leftBackPower, rightBackPower);

    }

    public void setDrivePower(double leftFront, double rightFront, double leftBack,
                              double rightBack) {
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
            myOpMode.telemetry.addData("Lift Current", "Left Lift: %4d Right Lift: %4d Power %1.2f", leftLift.getCurrentPosition(), rightLift.getCurrentPosition(), leftLift.getPower());
            myOpMode.telemetry.addData("Lift Target ", "Left Lift: %4d Right Lift: %4d Offset %3d", leftLift.getTargetPosition(), rightLift.getTargetPosition(), liftOffset);
            myOpMode.telemetry.addData("Lift Safety ", "Min: %4d  mMax: %4d Button On %s", liftHeightMin, liftHeightMax, !liftSafetyButton.getState());
            Pose2D pos = odo.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            myOpMode.telemetry.addData("Position", data);
            myOpMode.telemetry.addData("Drive Train", "LF %4d LB %4d RF %4d RB %4d Power Adj: ", leftFrontDrive.getCurrentPosition(), leftBackDrive.getCurrentPosition(), rightFrontDrive.getCurrentPosition(), rightBackDrive.getCurrentPosition(), MaxPowerAdjustment);
            myOpMode.telemetry.addData("Lift", "Left %3d  Right %3d  Arm %1.1f", leftLift.getCurrentPosition(), rightLift.getCurrentPosition(), liftArm.getPosition());
            myOpMode.telemetry.addData("Extension", "Ext %1.1f  Arm %1.1f  Spin %1.1f  Gripper %1.1f", extension.getPosition(), extensionArm.getPosition(), extensionSpin.getPosition(), extensionGripper.getPosition());

            myOpMode.telemetry.update();
            checkLiftPosition();
        } catch (Exception ex) {
            myOpMode.telemetry.addData("Error", "%s", ex.getMessage());
            myOpMode.telemetry.update();
        }
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
