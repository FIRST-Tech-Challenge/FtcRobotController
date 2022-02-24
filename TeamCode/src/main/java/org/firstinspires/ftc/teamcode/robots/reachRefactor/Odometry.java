package org.firstinspires.ftc.teamcode.robots.reachRefactor;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.robots.reachRefactor.util.Utils.wrapAngleRad;

@Config
@TeleOp(name = "AAAAOdometry test")
public class Odometry extends OpMode {

    DcMotorEx leftMotor, rightMotor;
    double headingOffset;
    boolean imuOffsetsInitialized;
    FtcDashboard dashboard;
    List<DcMotorEx> motors;
    BNO055IMU imu;

    public static double STARTING_HEADING = Math.toRadians(90);
    public static double STARTING_X = 12;
    public static double STARTING_Y = 72;
    public static double RADIUS = 6;
    public static double TICKS_PER_INCH = 30.7708333333;

    double x, y;
    double lastLeftPosition, lastRightPosition;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        leftMotor = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        rightMotor = hardwareMap.get(DcMotorEx.class, "motorFrontRight");

        motors = Arrays.asList(leftMotor, rightMotor);
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        x = STARTING_X;
        y = STARTING_Y;
    }

    private double wrapAngleRad(double angle){
        return ((angle % (2 * Math.PI)) + (2 * Math.PI)) % (2 * Math.PI);
    }

    @Override
    public void loop() {
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        leftMotor.setPower(left);
        rightMotor.setPower(right);

        double leftPosition = leftMotor.getCurrentPosition();
        double rightPosition = rightMotor.getCurrentPosition();

        Orientation orientation = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.RADIANS);
        if(!imuOffsetsInitialized && imu.isGyroCalibrated()) {
            headingOffset = wrapAngleRad(orientation.firstAngle);

            imuOffsetsInitialized = true;
        }

        double heading = wrapAngleRad(orientation.firstAngle - headingOffset + STARTING_HEADING);
        double leftDiff = leftPosition - lastLeftPosition;
        double rightDiff = rightPosition - lastRightPosition;
        double displacement = (leftDiff + rightDiff) / 2 / TICKS_PER_INCH;
        double dx = displacement * Math.cos(heading);
        double dy = displacement * Math.sin(heading);
        x += dx;
        y += dy;

        lastLeftPosition = leftPosition;
        lastRightPosition = rightPosition;

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", Math.toDegrees(heading));
        telemetry.addData("left ticks", leftPosition);
        telemetry.addData("right ticks", rightPosition);

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", x);
        packet.put("y", y);
        packet.put("heading", Math.toDegrees(heading));
        packet.put("left ticks", leftPosition);
        packet.put("right ticks", rightPosition);

        Canvas canvas = packet.fieldOverlay();
        canvas.strokeCircle(x, y, RADIUS);
        canvas.strokeLine(x, y, x + RADIUS * Math.cos(heading), y + RADIUS * Math.sin(heading));

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
