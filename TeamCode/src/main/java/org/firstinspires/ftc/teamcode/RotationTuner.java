package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

    private Orientation angles;
    private PIDController pidRotate;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePadController(gamepad1);

        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(2);

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
    }

    @Override
    public void loop() {
        gamepad.update();
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double power = pidRotate.calculate(angles.firstAngle, Vals.rotate_target);

        driveLeft.set(-power);
        driveRight.set(power);

        telemetry.addData("Current Heading", angles.firstAngle);
        telemetry.addData("Current Power", power);

    }

}
