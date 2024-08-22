/*
    SPDX-License-Identifier: MIT

    Copyright (c) 2024 SparkFun Electronics
*/
package com.qualcomm.hardware.sparkfun;

import java.nio.ByteBuffer;
import java.util.Arrays;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.wilyworks.simulator.WilyCore;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@I2cDeviceType
@DeviceProperties(
        name = "SparkFun OTOS",
        xmlTag = "SparkFunOTOS",
        description = "SparkFun Qwiic Optical Tracking Odometry Sensor"
)
public class SparkFunOTOS extends I2cDeviceSynchDevice<I2cDeviceSynch> {
    public SparkFunOTOS(I2cDeviceSynch deviceClient) {
        super(deviceClient, true);
    }

    public static final byte DEFAULT_ADDRESS = 0x17;
    public static final double MIN_SCALAR = 0.872;
    public static final double MAX_SCALAR = 1.127;

    // 2D pose structure, including x and y coordinates and heading angle.
    // Although pose is traditionally used for position and orientation, this
    // structure is also used for velocity and accleration by the OTOS driver
    public static class Pose2D {
        public double x;
        public double y;
        public double h;

        public Pose2D() {
            x = 0.0;
            y = 0.0;
            h = 0.0;
        }

        public Pose2D(double x, double y, double h) {
            this.x = x;
            this.y = y;
            this.h = h;
        }

        public void set(Pose2D pose) {
            this.x = pose.x;
            this.y = pose.y;
            this.h = pose.h;
        }
    }

    // Version register structure
    public static class Version {
        public byte minor;
        public byte major;

        public Version() {
            set((byte) 0);
        }

        public Version(byte value) {
            set(value);
        }

        public void set(byte value) {
            minor = (byte) (value & 0x0F);
            major = (byte) ((value >> 4) & 0x0F);
        }

        public byte get() {
            return (byte) ((major << 4) | minor);
        }
    }

    // Signal process config register structure
    public static class SignalProcessConfig {
        public boolean enLut;
        public boolean enAcc;
        public boolean enRot;
        public boolean enVar;

        public SignalProcessConfig() {
            set((byte) 0);
        }

        public SignalProcessConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            enLut = (value & 0x01) != 0;
            enAcc = (value & 0x02) != 0;
            enRot = (value & 0x04) != 0;
            enVar = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((enLut ? 0x01 : 0) | (enAcc ? 0x02 : 0) | (enRot ? 0x04 : 0) | (enVar ? 0x08 : 0));
        }
    }

    // Self-test config register structure
    public static class SelfTestConfig {
        public boolean start;
        public boolean inProgress;
        public boolean pass;
        public boolean fail;

        public SelfTestConfig() {
            set((byte) 0);
        }

        public SelfTestConfig(byte value) {
            set(value);
        }

        public void set(byte value) {
            start = (value & 0x01) != 0;
            inProgress = (value & 0x02) != 0;
            pass = (value & 0x04) != 0;
            fail = (value & 0x08) != 0;
        }

        public byte get() {
            return (byte) ((start ? 0x01 : 0) | (inProgress ? 0x02 : 0) | (pass ? 0x04 : 0) | (fail ? 0x08 : 0));
        }
    }

    // Status register structure
    public static class Status {
        public boolean warnTiltAngle;
        public boolean warnOpticalTracking;
        public boolean errorPaa;
        public boolean errorLsm;

        public Status() {
            set((byte) 0);
        }

        public Status(byte value) {
            set(value);
        }

        public void set(byte value) {
            warnTiltAngle = (value & 0x01) != 0;
            warnOpticalTracking = (value & 0x02) != 0;
            errorPaa = (value & 0x40) != 0;
            errorLsm = (value & 0x80) != 0;
        }

        public byte get() {
            return (byte) ((warnTiltAngle ? 0x01 : 0) | (warnOpticalTracking ? 0x02 : 0) | (errorPaa ? 0x40 : 0) | (errorLsm ? 0x80 : 0));
        }
    }

    protected DistanceUnit _distanceUnit = DistanceUnit.INCH;
    protected AngleUnit _angularUnit = AngleUnit.DEGREES;
    protected double _linearScalar = 0;
    protected double _angularScalar = 0;
    protected Pose2D _offset = new Pose2D(0, 0, 0);

    // Helpers to convert the OTOS notion of pose to-and-from the simulation's notion of pose:
    protected Pose2d simulationPose(Pose2D otosPose) {
        return new Pose2d(
                _distanceUnit.toInches(otosPose.x),
                _distanceUnit.toInches(otosPose.y),
                _angularUnit.toRadians(otosPose.h));
    }
    protected Pose2D otosPose(Pose2d simulationPose) {
        return new Pose2D(
                _distanceUnit.fromInches(simulationPose.position.x),
                _distanceUnit.fromInches(simulationPose.position.y),
                _angularUnit.fromRadians(simulationPose.heading.log()));
    }

    @Override
    protected boolean doInitialize() { return true; }

    @Override
    public HardwareDevice.Manufacturer getManufacturer() { return HardwareDevice.Manufacturer.SparkFun; }

    @Override
    public String getDeviceName()
    {
        return "SparkFun Qwiic Optical Tracking Odometry Sensor";
    }

    public boolean begin() { return isConnected(); }
    public boolean isConnected() { return true; }
    public void getVersionInfo(Version hwVersion, Version fwVersion) {
        hwVersion.set((byte) 0);
        fwVersion.set((byte) 0);
    }
    public boolean selfTest() { return true; }
    public boolean calibrateImu() {
        return calibrateImu(255, true);
    }
    public boolean calibrateImu(int numSamples, boolean waitUntilDone) { return true; }
    public int getImuCalibrationProgress() { return 0; }
    public DistanceUnit getLinearUnit() {
        return _distanceUnit;
    }
    public void setLinearUnit(DistanceUnit unit) { _distanceUnit = unit; }
    public AngleUnit getAngularUnit() {
        return _angularUnit;
    }
    public void setAngularUnit(AngleUnit unit) { _angularUnit = unit; }
    public double getLinearScalar() { return _linearScalar; }
    public boolean setLinearScalar(double scalar) {
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;
        _linearScalar = scalar;
        return true;
    }
    public double getAngularScalar() { return _angularScalar; }
    public boolean setAngularScalar(double scalar) {
        if (scalar < MIN_SCALAR || scalar > MAX_SCALAR)
            return false;
        _angularScalar = scalar;
        return true;
    }
    public void resetTracking() {}
    public SignalProcessConfig getSignalProcessConfig() { return new SignalProcessConfig((byte) 0); };
    public void setSignalProcessConfig(SignalProcessConfig config) { }
    public Status getStatus() { return new Status((byte) 0); }
    public Pose2D getOffset() { return new Pose2D(_offset.x, _offset.y, _offset.h); }
    public void setOffset(Pose2D pose) { _offset = new Pose2D(_offset.x, _offset.y, _offset.h); }
    public Pose2D getPosition() { return otosPose(WilyCore.getPose()); }
    public void setPosition(Pose2D pose) { WilyCore.setStartPose(simulationPose(pose), null); }
    public Pose2D getVelocity() { return new Pose2D(0, 0, 0); }
    public Pose2D getAcceleration() { return new Pose2D(0, 0, 0); }
    public Pose2D getPositionStdDev() { return new Pose2D(0, 0, 0); }
    public Pose2D getVelocityStdDev() { return new Pose2D(0, 0, 0); }
    public Pose2D getAccelerationStdDev() { return new Pose2D(0, 0, 0); }
    public void getPosVelAcc(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPosition());
        vel.set(getVelocity());
        acc.set(getAcceleration());
    }
    public void getPosVelAccStdDev(Pose2D pos, Pose2D vel, Pose2D acc) {
        pos.set(getPositionStdDev());
        vel.set(getVelocityStdDev());
        acc.set(getAccelerationStdDev());
    }
    public void getPosVelAccAndStdDev(Pose2D pos, Pose2D vel, Pose2D acc,
                                      Pose2D posStdDev, Pose2D velStdDev, Pose2D accStdDev) {
        getPosVelAcc(pos, vel, acc);
        getPosVelAccStdDev(posStdDev, velStdDev, accStdDev);
    }
    protected void poseToRegs(byte[] rawData, Pose2D pose, double xyToRaw, double hToRaw) { }
}
