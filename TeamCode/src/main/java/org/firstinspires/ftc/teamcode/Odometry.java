package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="Odometry Testing")
public class Odometry extends LinearOpMode {

    private final double TICKS_PER_REVOLUTION = 2000;
    private final double DIAMETER = 1.889764;
    private final double CIRCUMFERENCE = PI * DIAMETER;


    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftOdometry = null;
    private DcMotor rightOdometry = null;
    private DcMotor perpOdemetry = null;
    private IMU imu = null;

    public double getLeftOdometry() {
        return -leftOdometry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double getRightOdometry() {
        return rightOdometry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double getPerpOdometry() {
        return -perpOdemetry.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    public double[] getPosition() {
        double x = CIRCUMFERENCE * getPerpOdometry();
        double y = CIRCUMFERENCE * (getLeftOdometry() + getRightOdometry()) / 2;
        return new double[] {x, y};
    }

    public double getRotation() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void init_sensors() {
        leftOdometry = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightOdometry = hardwareMap.get(DcMotor.class, "right_front_drive");
        perpOdemetry = hardwareMap.get(DcMotor.class, "right_back_drive");
        imu = hardwareMap.get(IMU.class, "imu");
    }

    @Override
    public void runOpMode() {
        init_sensors();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            telemetry.addData("Left dead wheel position", leftOdometry.getCurrentPosition());
            telemetry.addData("Right dead wheel position", rightOdometry.getCurrentPosition());
            telemetry.addData("Strafe dead wheel position", perpOdemetry.getCurrentPosition());
            telemetry.addData("X Position", getPosition()[0]);
            telemetry.addData("Y Position", getPosition()[1]);
            telemetry.addData("Rotation", getRotation());
            telemetry.update();
        }
    }
}
