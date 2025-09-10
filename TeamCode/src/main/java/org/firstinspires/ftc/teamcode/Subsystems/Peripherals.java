
package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Tools.Mouse;

public class Peripherals extends Subsystem {

    private static DcMotor leftMotor;
    private static DcMotor rightMotor;
    private double xPosition = 0, yPosition = 0, theta = 0;
    private int lastLeftPosition = 0, lastRightPosition = 0;
    private double wheelDiameter = 0.1;
    private double wheelBase = 0.3;
    static IMU imu;
//    private static Limelight3A limelight;

    public Peripherals(String name) {
        super(name);
    }

    public static void initialize(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");

//        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        limelight.setPollRateHz(60);
//        limelight.pipelineSwitch(0);
//        limelight.getStatus();
//        limelight.start();
    }

    public static double getYawDegrees(){
        return Mouse.getTheta();
    }

    public static double getYaw() {
        return Math.toRadians(Mouse.getTheta());
    }

    public static double getRoll() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public static double getPitch() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public static void resetYaw() {
        imu.resetYaw();
    }

//    public static LLResult getLimelightResult() {
//        return limelight.getLatestResult();
//    }
//
//    public static void updateRobotOrientation(double yaw) {
//        limelight.updateRobotOrientation(yaw);
//    }
//
//    public static void stopLimelight() {
//        limelight.stop();
//    }
//
//
//    public static double getLimelightX() {
//        LLResult result = getLimelightResult();
//        if (result != null && result.isValid()) {
//            return result.getBotpose().getPosition().x + 1.83;
//        }
//        return 0;
//    }
//
//
//    public static double getLimelightY() {
//        LLResult result = getLimelightResult();
//        if (result != null && result.isValid()) {
//            return result.getBotpose().getPosition().y + 1.83;
//        }
//        return 0;
//    }
}
