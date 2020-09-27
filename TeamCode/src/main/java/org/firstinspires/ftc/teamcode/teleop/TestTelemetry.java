package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.hardware.TestHardware;

@TeleOp(name="Localizer Telemetry")
public class TestTelemetry extends TestHardware {

    @Override
    public void init() {
        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        this.revIMU.initialize(parameters);
        this.revIMU.startAccelerationIntegration(new Position(), new Velocity(), 10);
    }



    @SuppressLint("DefaultLocale")
    @Override
    public void loop() {
        telemetry.addData("gyro calib", this.revIMU.isGyroCalibrated());
        telemetry.addData("accel calib", this.revIMU.isAccelerometerCalibrated());
        telemetry.addData("magmo calib", this.revIMU.isMagnetometerCalibrated());
        telemetry.addData("calib", this.revIMU.getCalibrationStatus().toString());

        Orientation orientation = this.revIMU.getAngularOrientation();
        telemetry.addData("angle", String.format("%f %f %f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle));

        Acceleration acceleration = this.revIMU.getLinearAcceleration();
        telemetry.addData("accel", String.format("%f %f %f", acceleration.xAccel, acceleration.yAccel, acceleration.zAccel));

        Velocity velocity = this.revIMU.getVelocity();
        telemetry.addData("velocity", String.format("%f %f %f", velocity.xVeloc, velocity.yVeloc, velocity.zVeloc));

        Position imuPosition = this.revIMU.getPosition();
        telemetry.addData("XYZ", String.format("%f %f %f",
                imuPosition.x,
                imuPosition.y,
                imuPosition.z));

    }
}
