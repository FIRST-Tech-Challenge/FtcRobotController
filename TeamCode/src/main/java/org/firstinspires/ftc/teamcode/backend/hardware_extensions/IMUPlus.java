package org.firstinspires.ftc.teamcode.backend.hardware_extensions;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.backend.control.low_level.PIDV2;

public class IMUPlus {
    double kp, ki, kd;
    PIDV2 imu_pid;

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    private double                  globalAngle;

    private double correction = 0.0;

    public IMUPlus(BNO055IMU imu, double[] imu_pid_values){
        this.imu = imu;

        this.kp = imu_pid_values[0];
        this.ki = imu_pid_values[1];
        this.kd = imu_pid_values[2];

        imu_pid = new PIDV2(kp, ki, kd) {
            @Override
            public void perform(double response) {
                report_correction(response);
            }

            @Override
            public double getInputData() {
                return getAngle();
            }
        };
    }

    public boolean isGyroCalibrated(){
        return imu.isGyroCalibrated();
    }

    private void report_correction(double correction){
        this.correction = correction;
    }

    public void resetAngle(){
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        globalAngle = 0;
        imu_pid.restartPID();
    }

    public double getAngle(){
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180){
            deltaAngle += 360;
        } else if (deltaAngle > 180){
            deltaAngle -= 360;
        }
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }

    public void imu_iter(double target_angle){
        imu_pid.executePID(target_angle);
    }

    public double getCorrection() {
        return correction;
    }

    public double getGlobalAngle() {
        return globalAngle;
    }
}
