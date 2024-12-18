package org.firstinspires.ftc.teamcode.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
public class TubeDriver {
    public static double ROTATION_P = 0.013;
    public static double ROTATION_I = 0.008;    // (0.006, Dec 7, 2023)
    public static double ROTATION_D = 0;
    public static double ROTATION_T = 0.5;
    public static double ROTATION_PID_POWER_LIMIT = 0.5;
    public static double ROTATION_Kx = -0.13;
    public static double ROTATION_Kv = 0.25;
    public static double ROTATION_I_MAX = 0.2;

    public static int ROTATION_OFFSET_TICKS = 1090;
    public static double TICKS_PER_TUBE_ROTATION_2 = 8192;


    public final org.firstinspires.ftc.teamcode.Arm.SlideVerticalArm slideVerticalArm;

    /**
     * Slide motors are set to RUN_WITHOUT_ENCODER.
     *
     * @param rotationMotor1  Rotation DcMotorEx with no reversing.
     */
    public TubeDriver(DcMotorEx rotationMotor1, DcMotorEx rotationMotor2) {
        slideVerticalArm = new org.firstinspires.ftc.teamcode.Arm.SlideVerticalArm(rotationMotor1, rotationMotor2);
        slideVerticalArm.resetPidValues();
    }

    public void update() {
        slideVerticalArm.updateMotorPower();
    }

    /**
     * Set the target position of the Slide_Motors in inches.
     * Set the target position of the Rotation-Motor in degrees.
     *
     * @param extensionPosition_ticks The length of the target position in ticks.
     * @param rotationAngle_degrees   The angle of the target position in degrees.
     */
    public void setArmPosition(int extensionPosition_ticks, double rotationAngle_degrees) {
        slideVerticalArm.setToAngleDegrees(rotationAngle_degrees);
    }

    public double getCurrentAngleDegrees() {
        return slideVerticalArm.getCurrentAngleDegrees();
    }

    public double getTargetAngle() {
        return slideVerticalArm.rotationTargetAngle;
    }

    public static void initExtensionMotors(DcMotorEx extensionMotor1, DcMotorEx extensionMotor2) {
        extensionMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        extensionMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        extensionMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        extensionMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public static void initRotationMotors(DcMotorEx rotationMotor1, DcMotorEx rotationMotor2) {
        rotationMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
        rotationMotor2.setDirection(DcMotorSimple.Direction.FORWARD);

        rotationMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rotationMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void stopAllMotors() {
        slideVerticalArm.stopMotors();
    }

    public void rotateUp() {
        slideVerticalArm.manualControl(-1);
    }
}
