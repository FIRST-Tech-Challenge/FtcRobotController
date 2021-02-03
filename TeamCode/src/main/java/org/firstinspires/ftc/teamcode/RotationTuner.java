package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "RotationTuner", group = "Tuner")
public class RotationTuner extends OpMode {

    private GamePadController gamepad;
    private Motor driveLeft, driveRight;
    private BNO055IMU imu;

    private Orientation lastAngles;
    private double currentHeading = 0;
    private PIDController pidRotate;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePadController(gamepad1);

        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(Vals.rotate_tolerance);

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveRight.setInverted(true);
        driveLeft.setRunMode(Motor.RunMode.RawPower);
        driveRight.setRunMode(Motor.RunMode.RawPower);
//        driveLeft.setVeloCoefficients(0.05, 0, 0);
//        driveRight.setVeloCoefficients(0.05, 0, 0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }

    @Override
    public void loop() {
        gamepad.update();
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        pidRotate.setPID(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
//        double power = pidRotate.calculate(lastAngle.firstAngle, Vals.rotate_target);
//
//
//
//        driveLeft.set(-power);
//        driveRight.set(power);
//
//        telemetry.addData("Current Heading", lastAngles.firstAngle);
//        telemetry.addData("Current Power", power);

        if(gamepad.isARelease()) {
            rotate(Vals.rotate_target);
        }

        telemetry.addData("Current Heading", lastAngles.firstAngle);
        telemetry.addData("Current Heading", updateHeading());
        telemetry.addData("Current Power", driveLeft.get());

    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
    }

    private double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        } else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        currentHeading += deltaAngle;

        return currentHeading;
    }

    private void rotate(double degrees) {
        resetAngle();

        if(Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        pidRotate.reset();
        ElapsedTime elapsedTime = new ElapsedTime();

        pidRotate.setSetPoint(degrees);

        while(!pidRotate.atSetPoint() && elapsedTime.seconds() < 2) {
            double power = pidRotate.calculate(updateHeading());

            driveLeft.set(-power);
            driveRight.set(power);

            telemetry.addData("Current Angle", lastAngles.firstAngle);
            telemetry.addData("Current Heading", currentHeading);
            telemetry.addData("Current Power", power);
            telemetry.update();
        }

        driveLeft.set(0);
        driveRight.set(0);

    }

}
