/*
 Copyright (c) 2020 HF Robotics (http://www.hfrobots.com)

 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
 */

package com.hfrobots.tnt.corelib.drive.mecanum;

import android.util.Log;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.quickstart.util.AxesSigns;
import com.acmerobotics.roadrunner.quickstart.util.BNO055IMUUtil;
import com.google.common.base.Optional;
import com.hfrobots.tnt.corelib.util.LynxModuleUtil;
import com.hfrobots.tnt.corelib.util.SimplerHardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import lombok.NonNull;

import static com.acmerobotics.roadrunner.quickstart.drive.DriveConstants.encoderTicksToInches;
import static com.hfrobots.tnt.corelib.Constants.LOG_TAG;

/*
 * Simple mecanum drive hardware implementation for REV hardware. If your hardware configuration
 * satisfies the requirements, SampleMecanumDriveREVOptimized is highly recommended.
 */
public class RoadRunnerMecanumDriveREV extends RoadRunnerMecanumDriveBase {
    private DcMotorEx leftFront, leftRear, rightRear, rightFront;

    private List<DcMotorEx> motors;

    private BNO055IMU imu;

    private boolean encodersEnabled = false;

    public RoadRunnerMecanumDriveREV(final DriveConstants driveConstants, SimplerHardwareMap hardwareMap,
                                     boolean needImu) {
        this(driveConstants, hardwareMap, needImu, Optional.<AxesOrder>absent(), Optional.<AxesSigns>absent());

    }

    public RoadRunnerMecanumDriveREV(final DriveConstants driveConstants,
                                     SimplerHardwareMap hardwareMap,
                                     boolean needImu,
                                     Optional<AxesOrder> optionalAxesOrder,
                                     Optional<AxesSigns> optionalAxesSigns) {
        super(driveConstants);

        LynxModuleUtil.ensureMinimumFirmwareVersion(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        if (needImu) {
            // TODO: adjust the names of the following hardware devices to match your configuration
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);

            if (optionalAxesOrder.isPresent() && optionalAxesSigns.isPresent()) {
                // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
                // upward (normal to the floor) using a command like the following:
                BNO055IMUUtil.remapAxes(imu, optionalAxesOrder.get(), optionalAxesSigns.get());
            }
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "leftFrontDriveMotor");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRearDriveMotor");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRearDriveMotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFrontDriveMotor");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        encodersEnabled = true;

        for (DcMotorEx motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TO-DONE: reverse any motors using DcMotor.setDirection()
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        PIDFCoefficients coefficients = leftFront.getPIDFCoefficients(runMode);
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, new PIDFCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD, 1
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : motors) {
            wheelPositions.add(encoderTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    @Override
    public void setMotorPowers(double v, double v1, double v2, double v3) {
        leftFront.setPower(v);
        leftRear.setPower(v1);
        rightRear.setPower(v2);
        rightFront.setPower(v3);
    }

    @Override
    public double getRawExternalHeading() {
        if (imu != null) {
            Orientation orientation = imu.getAngularOrientation();

            double angle = orientation.firstAngle;

            Log.d("RR", "imu-angles:" + orientation.firstAngle + ", " + orientation.secondAngle + ", " + orientation.thirdAngle);
            Log.d( "RR", "imu axes order: " + orientation.axesOrder + " in " + orientation.angleUnit);

            return angle;
        }

        return 0;
    }

    public void enableEncoders() {

        if (encodersEnabled) {
            return; // don't do it again if not needed
        }

        Log.d(LOG_TAG, "Enabling encoders for the drivebase");

        for (DcMotorEx motor : motors) {
           motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
           motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        encodersEnabled = true;
    }

    public void disableEncoders() {

        if (!encodersEnabled) {
            return; // don't do it again if not needed
        }

        Log.d(LOG_TAG, "Disabling encoders for the drivebase");

        for (DcMotorEx motor : motors) {
           motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        encodersEnabled = false;
    }
}
