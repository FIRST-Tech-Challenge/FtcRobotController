/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveTrain {

    public DcMotorEx left_front;
    public DcMotorEx right_front;
    public DcMotorEx left_back;
    public DcMotorEx right_back;

    static final double DRIVE_SPEED = 40000;
    static final double TURN_SPEED_MODIFIER = 0.7;

    HardwareMap hardwareMap;

    public void init(HardwareMap ahwMap) {
        hardwareMap = ahwMap;

        left_front = hardwareMap.get(DcMotorEx.class, "left_front");
        right_front = hardwareMap.get(DcMotorEx.class, "right_front");
        left_back = hardwareMap.get(DcMotorEx.class, "left_back");
        right_back = hardwareMap.get(DcMotorEx.class, "right_back");

        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Key: velocities = [left_front, right_front, left_back, right_back]
    public void setDriveVelocities(double[] velocities) {
        left_front.setVelocity(velocities[0]);
        right_front.setVelocity(velocities[1]);
        left_back.setVelocity(velocities[2]);
        right_back.setVelocity(velocities[3]);
    }

    public double[] drive(double leftStickX, double leftStickY, double rightStickX){

        double r = Math.hypot(leftStickX, leftStickY);
        rightStickX *= TURN_SPEED_MODIFIER;


        double robotAngle = Math.atan2(-leftStickY, leftStickX)
                - Math.PI / 4;
        //- this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double v1 = r * Math.cos(robotAngle) + (double) rightStickX;
        double v2 = r * Math.sin(robotAngle) - (double) rightStickX;
        double v3 = r * Math.sin(robotAngle) + (double) rightStickX;
        double v4 = r * Math.cos(robotAngle) - (double) rightStickX;

        double[] velocities = new double[]{v1, v2, v3, v4};
        double maxVal = Utility.max(velocities);

        if (Math.abs(maxVal) > 1) {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] /= maxVal;
                velocities[i] *= DRIVE_SPEED;
            }
        }
        else {
            for (int i = 0; i < velocities.length; i++) {
                velocities[i] *= DRIVE_SPEED;
            }
        }

        return velocities;
    }

    public void brake() {
        left_front.setVelocity(0);
        right_front.setVelocity(0);
        left_back.setVelocity(0);
        right_back.setVelocity(0);
    }
}

*/

