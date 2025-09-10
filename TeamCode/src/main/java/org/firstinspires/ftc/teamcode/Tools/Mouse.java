package org.firstinspires.ftc.teamcode.Tools;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Mouse {
    private static double fieldX;
    private static double fieldY;
    private static double theta;
    private static SparkFunOTOS mouse;
    private static SparkFunOTOS.Pose2D field;

    public static void init(HardwareMap hardwareMap) {
        mouse = hardwareMap.get(SparkFunOTOS.class, "mouse");
    }
    public static void configureOtos() {

        mouse.setLinearUnit(SparkFunOTOS.LinearUnit.METERS);
        mouse.setAngularUnit(SparkFunOTOS.AngularUnit.DEGREES);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.15875, 0, -270);
        mouse.setOffset(offset);
        mouse.setLinearScalar(1.005809562240364);
        mouse.setAngularScalar(0.989932511851);
        mouse.calibrateImu();
        mouse.resetTracking();
        System.out.println("configed");
        System.out.println("theta"+offset.h);

    }
    public static void setPosition(double x, double y, double theta){
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(x, y, theta);
        mouse.setPosition(currentPosition);
    }
    public static void update() {
        field = mouse.getPosition();
        fieldX = field.x;
        fieldY =field.y;
        theta = field.h;
    }
    public static double getX() {
        return -fieldX;
    }
    public static double getY() {
        return fieldY;
    }
    public static double getTheta() {
        return theta;
    }
}