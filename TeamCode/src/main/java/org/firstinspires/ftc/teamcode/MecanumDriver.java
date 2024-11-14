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
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static final double MAX_EXTENSION_BACK = 6.0;
    public static final double MAX_EXTENSION_FORWARD = 15.0;
    public static int PIVOT_PICKUP_SPECIMEN = -490;
    public static int VIPER_PICKUP_SPECIMEN = ViperSlide.MIN_POSITION;
    // About 60 degrees
    public static int PIVOT_PLACE_SPECIMEN = -1160;
    public static int VIPER_PLACE_SPECIMEN = 1200;
    public static int PIVOT_POSITION_OFFSET_COUNTS = 1375;
    public final boolean isFieldCentric = true;
//    private Servo claw;
//    private Servo wrist;
    private ViperSlide viperSlide;
    private Pivot pivot;
    private Intake intake;
    private double pivotStartedTime;
    private boolean pivotStarted;
//    private boolean clawOpen;
    private SparkFunOTOS photoSensor;

    @Override
    public void init() {
        DcMotorEx backLeft = hardwareMap.get(DcMotorEx.class, "BACKLEFT");
        DcMotorEx backRight = hardwareMap.get(DcMotorEx.class, "BACKRIGHT");
        DcMotorEx frontLeft = hardwareMap.get(DcMotorEx.class, "FRONTLEFT");
        DcMotorEx frontRight = hardwareMap.get(DcMotorEx.class, "FRONTRIGHT");
////        claw = hardwareMap.get(Servo.class, "CLAWLEFT");
////        claw.setDirection(Servo.Direction.REVERSE);
////        wrist = hardwareMap.get(Servo.class, "CLAWRIGHT");
//
        IMU gyro = hardwareMap.get(IMU.class, "imu2");
//
        SparkFunOTOS photoSensor = hardwareMap.get(SparkFunOTOS.class, "PHOTOSENSOR");
        this.photoSensor = photoSensor;

        configureOtos();

//        this.photoSensor.setLinearUnit(DistanceUnit.INCH);
//        this.photoSensor.setAngularUnit(AngleUnit.DEGREES);
//
////        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.142, 0, 180);
////        this.photoSensor.setOffset(offset);
//
//        this.photoSensor.setLinearScalar(1.0);
//        this.photoSensor.setAngularScalar(1.0);
//
//        this.photoSensor.calibrateImu();
//        this.photoSensor.resetTracking();
//
//        viperSlide = new ViperSlide(
//                hardwareMap.get(DcMotorEx.class, "VIPERLEFT"),
//                hardwareMap.get(DcMotorEx.class, "VIPERRIGHT")
//        );
//        pivot = new Pivot(
//                hardwareMap.get(DcMotorEx.class, "PIVOTLEFT"),
//                hardwareMap.get(DcMotorEx.class, "PIVOTRIGHT"),
//                PIVOT_POSITION_OFFSET_COUNTS
//        );
//        intake = new Intake(hardwareMap);
        robotController = new MecanumRobotController(backLeft, backRight, frontLeft, frontRight, gyro, photoSensor);
//
//        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        runtime.reset();
//        intake.close();
//        intake.unwhack();
//        claw.setPosition(0.75);
//        wrist.setPosition(WRIST_MIN_POS);
    }

    @Override
    public void loop() {
        // Drivetrain
//        robotController.continuousDrive(gamepad1.left_stick_y * SPEED_MULTIPLIER * FORWARD_POWER,
//                gamepad1.left_stick_x * SPEED_MULTIPLIER * STRAFE_POWER,
//                gamepad1.right_stick_x * TURN_POWER, isFieldCentric);
//        double extensionBeyondChassis = viperSlide.getExtensionBeyondChassis(pivot.getAngleDegrees());
//
//        double pivotPower = Math.min(MAX_PIVOT_VELOCITY, BASE_PIVOT_VELOCITY + (MAX_PIVOT_VELOCITY - BASE_PIVOT_VELOCITY) * (runtime.seconds() - pivotStartedTime) / PIVOT_RAMP_TIME);
//        double currentLimit = MAX_EXTENSION_FORWARD;
//        if (pivot.getAngleDegrees() > 30) {
//            currentLimit -= 1;
//        }
//        if (pivot.getAngleDegrees() > 40) {
//            currentLimit -= 1;
//        }
//        // Pivot
//        if (gamepad2.dpad_down /*&& extensionBeyondChassis < currentLimit - 0.75*/) {
//            if (!pivotStarted) {
//                pivotStartedTime = runtime.seconds();
//                pivotStarted = true;
//            }
//            pivot.move(pivotPower);
//        } else if (gamepad2.dpad_up /*&& extensionBeyondChassis > -MAX_EXTENSION_BACK + 0.75 && !(pivot.getAngleDegrees() > 100)*/) {
//            if (!pivotStarted) {
//                pivotStartedTime = runtime.seconds();
//                pivotStarted = true;
//            }
//            pivot.move(-pivotPower);
//        } else {
//            pivot.hold();
//            pivotStarted = false;
//        }
//
//        double triggerPower = gamepad2.left_trigger - gamepad2.right_trigger;
//        // Viper
//    if (triggerPower != 0 /*|| (triggerPower != 0 && extensionBeyondChassis < MAX_EXTENSION_FORWARD && extensionBeyondChassis > -MAX_EXTENSION_BACK) && pivot.getAngleDegrees() < 110*/) {
//            viperSlide.move(triggerPower * VIPER_VELOCITY_CONSTANT);
//        } else {
//            viperSlide.hold();
//        }
//
//        // Preset Viper Positions
//        // Currently disabled to make sure no extension beyond limits
//        /*if (gamepad2.left_bumper) {
//            viperSlide.setTargetPosition(ViperSlide.MAX_POSITION);
//        } else*/ if (gamepad2.right_bumper) {
//            viperSlide.setTargetPosition(ViperSlide.MIN_POSITION);
//        }
//
//        // Open/close intake
//        if (gamepad2.a && runtime.seconds() - timeAPressed >= INTAKE_COOLDOWN) {
//            telemetry.addData("Button", "A pressed");
//            timeAPressed = runtime.seconds();
//            if (intake.isOpen()/*clawOpen*/) {
//                intake.close();
////                claw.setPosition(0.75);
////                clawOpen = false;
//            } else {
//                intake.open();
////                claw.setPosition(0.65);
////                clawOpen = true;
//            }
//        }
//
//        if (gamepad2.x && runtime.seconds() - timeXPressed >= WHACK_COOLDOWN) {
//            timeXPressed = runtime.seconds();
//            if (intake.isWhacked()) {
//                intake.unwhack();
//            } else {
//                intake.whack();
//            }
//        }
//
//        // Pickup Preset
//        if (gamepad2.b) {
//            pivot.setTargetPosition(PIVOT_PICKUP_SPECIMEN);
//            viperSlide.setTargetPosition(VIPER_PICKUP_SPECIMEN);
//        }
//
//        // Place Preset
//        if (gamepad2.y) {
//            pivot.setTargetPosition(PIVOT_PLACE_SPECIMEN);
//            viperSlide.setTargetPosition(VIPER_PLACE_SPECIMEN);
//        }
//
//        if (gamepad2.left_bumper) {
//            pivot.setTargetPosition(-1700);
//            viperSlide.setTargetPosition(1900);
//        }

//        if (gamepad2.dpad_left) {
//            wrist.setPosition(Math.max(WRIST_MIN_POS, wrist.getPosition() - 0.03));
//        } else if (gamepad2.dpad_right) {
//            wrist.setPosition(Math.min(WRIST_MAX_POS, wrist.getPosition() + 0.03));
//        }

        SparkFunOTOS.Pose2D position = photoSensor.getPosition();
        for (int i = 0; i < 10; ++i) {
            robotController.turnTo(180, TURN_POWER);
            robotController.turnTo(-180, TURN_POWER);
            robotController.turnTo(0, TURN_POWER);
        }


        telemetry.addData("Position", position.x + ", " + position.y);
        telemetry.addData("Rotation", position.h);
//        telemetry.addData("", "");
//        telemetry.addData("Claw Left Position", intake.getLeftPosition());
//        telemetry.addData("Claw Right Position", intake.getRightPosition());
////        telemetry.addData("Claw Position", claw.getPosition());
////        telemetry.addData("Wrist Position", wrist.getPosition());
//        telemetry.addData("Viper Slide Position", viperSlide.getCurrentPosition());
//        telemetry.addData("Viper Extension Beyond Chassis", extensionBeyondChassis);
//        telemetry.addData("Viper Left Pos", viperSlide.getLeftPosition());
//        telemetry.addData("Viper Right Pos", viperSlide.getRightPosition());
//        telemetry.addData("Pivot Position", pivot.getCurrentPosition());
//        telemetry.addData("Pivot Angle", pivot.getAngleDegrees());
//        telemetry.addData("Whacker Position", intake.getWhackerPosition());
        telemetry.addData("", "");
//        robotController.sendTelemetry(telemetry);
    }

    private void configureOtos() {
        telemetry.addLine("Configuring OTOS...");
        telemetry.update();

        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        photoSensor.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        photoSensor.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        photoSensor.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        photoSensor.setLinearScalar(1.0);
        photoSensor.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        photoSensor.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        photoSensor.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        photoSensor.setPosition(currentPosition);
        photoSensor.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte) 0x0D));

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        photoSensor.getVersionInfo(hwVersion, fwVersion);

        telemetry.addLine("OTOS configured! Press start to get position data!");
        telemetry.addLine();
        telemetry.addLine(String.format("OTOS Hardware Version: v%d.%d", hwVersion.major, hwVersion.minor));
        telemetry.addLine(String.format("OTOS Firmware Version: v%d.%d", fwVersion.major, fwVersion.minor));
        telemetry.update();
    }
}
