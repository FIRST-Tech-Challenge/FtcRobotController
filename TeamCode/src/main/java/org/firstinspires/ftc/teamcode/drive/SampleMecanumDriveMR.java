package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.*;

import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/*
 * Simple mecanum drive hardware implementation for Modern Robotics hardware.
 */
public class SampleMecanumDriveMR extends SampleMecanumDriveBase {
    /*
     * As you may know, the MR communication system is implemented asynchronously. Thus, all
     * hardware calls return immediately (reads are cached and writes are queued). To ensure that
     * Road Runner isn't needlessly running on stale data (this is actually harmful), we delay after
     * each call to setMotorPowers() with the hope that, in most cases, new data will be available
     * by the next iteration (though this can never be guaranteed). Although it may seem attractive
     * to decrease this number and increase your control loop frequency, do so at your own risk.
     */
    private static final int MOTOR_WRITE_DELAY = 20;

    private DcMotor leftFront, leftRear, rightRear, rightFront;
    private List<DcMotor> motors;
    private BNO055IMU imu;

    public SampleMecanumDriveMR(HardwareMap hardwareMap) {
        super();

        // TODO: adjust the names of the following hardware devices to match your configuration
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        // TODO: if your hub is mounted vertically, remap the IMU axes so that the z-axis points
        // upward (normal to the floor) using a command like the following:
        // BNO055IMUUtil.remapAxes(imu, AxesOrder.XYZ, AxesSigns.NPN);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotor motor : motors) {
            // TODO: decide whether or not to use the built-in velocity PID
            // if you keep it, then don't tune kStatic or kA
            // otherwise, comment out the following line
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // TODO: reverse any motors using DcMotor.setDirection()

        // TODO: set the tuned coefficients from DriveVelocityPIDTuner if using RUN_USING_ENCODER
        // setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, ...);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(DcMotor.RunMode runMode) {
        ModernRoboticsUsbDcMotorController controller = (ModernRoboticsUsbDcMotorController) leftFront.getController();
        DifferentialControlLoopCoefficients coefficients = controller.getDifferentialControlLoopCoefficients(leftFront.getPortNumber());
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

    @NotNull
    @Override
    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotor motor : motors) {
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
