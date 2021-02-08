package org.firstinspires.ftc.teamcode.teleop;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.hardware.MovementHardware;

@TeleOp(name="Localizer Telemetry")
public class LocalizerTelemetry extends MovementHardware {

    @Override
    public void init() {
        super.init();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        this.revIMU.initialize(parameters);
        this.localizer.loadUltimateGoalTrackables(this);
    }


    @SuppressLint("DefaultLocale")
    @Override
    public void run_loop() {
        Position position = localizer.getEstimatedPosition().position;
        Orientation orientation = localizer.getEstimatedOrientation().orientation;
        if (position != null) {
            telemetry.addData("Localizer Position", String.format("%.1f, %.1f, %.1f", position.x, position.y, position.z));
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", orientation.firstAngle, orientation.secondAngle, orientation.thirdAngle);
        }

        telemetry.addData("IMU Gyro Calib", this.revIMU.isGyroCalibrated());
        Orientation imuAngularOrientation = this.revIMU.getAngularOrientation();
        telemetry.addData("IMU Angle", String.format("%f %f %f", imuAngularOrientation.firstAngle, imuAngularOrientation.secondAngle, imuAngularOrientation.thirdAngle));
        Acceleration acceleration = this.revIMU.getLinearAcceleration();
        telemetry.addData("IMU Acceleration", String.format("%f %f %f", acceleration.xAccel, acceleration.yAccel, acceleration.zAccel));


    }
}
