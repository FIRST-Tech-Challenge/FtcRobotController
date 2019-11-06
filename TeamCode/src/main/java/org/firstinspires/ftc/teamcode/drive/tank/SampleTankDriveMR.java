package org.firstinspires.ftc.teamcode.drive.tank;

import android.support.annotation.NonNull;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.encoderTicksToInches;
/*
 * Simple tank drive hardware implementation for Modern Robotics hardware.
 */
public class SampleTankDriveMR extends SampleTankDriveBase {
    /*
     * As you may know, the MR communication system is implemented asynchronously. Thus, all
     * hardware calls return immediately (reads are cached and writes are queued). To ensure that
     * Road Runner isn't needlessly running on stale data (this is actually harmful), we delay after
     * each call to setMotorPowers() with the hope that, in most cases, new data will be available
     * by the next iteration (though this can never be guaranteed). Although it may seem attractive
     * to decrease this number and increase your control loop frequency, do so at your own risk.
     */
    private static final int MOTOR_WRITE_DELAY = 20;

    private List<DcMotor> motors, leftMotors, rightMotors;
    private BNO055IMU imu;

    public SampleTankDriveMR(HardwareMap hardwareMap) {
        super();

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        // add/remove motors depending on your robot (e.g., 6WD)
        DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
        DcMotor leftRear = hardwareMap.dcMotor.get("leftRear");
        DcMotor rightRear = hardwareMap.dcMotor.get("rightRear");
        DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);
        leftMotors = Arrays.asList(leftFront, leftRear);
        rightMotors = Arrays.asList(rightFront, rightRear);

        for (DcMotor motor : motors) {
            if (RUN_USING_ENCODER) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        if (RUN_USING_ENCODER && MOTOR_VELO_PID != null) {
            setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, MOTOR_VELO_PID);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: if desired, use setLocalizer() to change the localization method
        // for instance, setLocalizer(new ThreeTrackingWheelLocalizer(...));
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        DcMotor leftFirst = leftMotors.get(0);
        ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) leftFirst.getController();
        DifferentialControlLoopCoefficients coefficients = controller.getDifferentialControlLoopCoefficients(leftFirst.getPortNumber());
        return new PIDCoefficients(coefficients.p, coefficients.i, coefficients.d);
    }

    @Override
    public void setPIDCoefficients(DcMotor.RunMode runMode, PIDCoefficients coefficients) {
        for (DcMotor motor : motors) {
            ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) motor.getController();
            controller.setDifferentialControlLoopCoefficients(motor.getPortNumber(), new DifferentialControlLoopCoefficients(
                    coefficients.kP, coefficients.kI, coefficients.kD
            ));
        }
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        double leftSum = 0, rightSum = 0;
        for (DcMotor leftMotor : leftMotors) {
            leftSum += encoderTicksToInches(leftMotor.getCurrentPosition());
        }
        for (DcMotor rightMotor : rightMotors) {
            rightSum += encoderTicksToInches(rightMotor.getCurrentPosition());
        }
        return Arrays.asList(leftSum / leftMotors.size(), rightSum / rightMotors.size());
    }

    @Override
    public void setMotorPowers(double v, double v1) {
        for (DcMotor leftMotor : leftMotors) {
            leftMotor.setPower(v);
        }
        for (DcMotor rightMotor : rightMotors) {
            rightMotor.setPower(v1);
        }

        try {
            Thread.sleep(MOTOR_WRITE_DELAY);
        } catch (InterruptedException e) {
            // do nothing
        }
    }

    @Override
    public double getRawExternalHeading() {
        return imu.getAngularOrientation().firstAngle;
    }
}
