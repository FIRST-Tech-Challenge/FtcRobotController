package org.firstinspires.ftc.teamcode.ultimategoal.modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ultimategoal.util.FileDumpProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.util.TelemetryProvider;
import org.firstinspires.ftc.teamcode.ultimategoal.Robot;

import java.util.ArrayList;
import java.util.Locale;

import static org.firstinspires.ftc.teamcode.ultimategoal.util.StringHelper.concat;

public class OdometryModule implements Module, TelemetryProvider, FileDumpProvider {
    private boolean isOn;

    public double worldX;
    public double worldY;
    public double worldAngleRad;

    private Robot robot;

    private DcMotor yLeft;
    private DcMotor yRight;
    private DcMotor mecanum;

    private final double INCHES_PER_ENCODER_TICK = 0.0007284406721 * 100.0/101.9889;
    private final double LR_ENCODER_DIST_FROM_CENTER = 6.942654509 * 3589.8638/3600.0 * 3531.4628211/3600.0;
    private final double M_ENCODER_DIST_FROM_CENTER = 4.5;

    private double leftPodOldPosition = 0;
    private double rightPodOldPosition = 0;
    private double mecanumPodOldPosition = 0;

    public double leftPodNewPosition;
    public double rightPodNewPosition;
    public double mecanumPodNewPosition;

    public OdometryModule(Robot robot, boolean isOn) {
        robot.telemetryDump.registerProvider(this);
        this.robot = robot;
        this.isOn = isOn;
    }

    public void init() {
        yLeft = robot.getDcMotor("yLeft");
        yRight = robot.getDcMotor("yRight");
        mecanum = robot.getDcMotor("mecanum");

        yLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanum.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        yLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        mecanum.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public synchronized void update() {
        calculateRobotPosition();
    }

    public ArrayList<String> getTelemetryData() {
        ArrayList<String> data = new ArrayList<>();
        data.add("worldX: " + worldX);
        data.add("worldY: " + worldY);
        data.add("heading: " + worldAngleRad);
        return data;
    }

    public String getFileName() {
        return System.currentTimeMillis() + "odometry.txt";
    }

    public String getFileData() {
        return String.format(Locale.CANADA_FRENCH, "(%f, %f), %f", worldX, worldY, worldAngleRad);
    }

    /**
     * Calculates the robot's position.
     */
    private void calculateRobotPosition() {
        leftPodNewPosition = yLeft.getCurrentPosition() * -1;
        rightPodNewPosition = yRight.getCurrentPosition();
        mecanumPodNewPosition = mecanum.getCurrentPosition();

        double leftPodDelta = leftPodNewPosition - leftPodOldPosition;
        double rightPodDelta = rightPodNewPosition - rightPodOldPosition;
        double mecanumPodDelta = mecanumPodNewPosition - mecanumPodOldPosition;

        calculateNewPosition(leftPodDelta, rightPodDelta, mecanumPodDelta);

        leftPodOldPosition = leftPodNewPosition;
        rightPodOldPosition = rightPodNewPosition;
        mecanumPodOldPosition = mecanumPodNewPosition;
    }

    public void calculateNewPosition(double dLeftPod, double dRightPod, double dMecanumPod) {
        // convert all inputs to inches
        double dLeftPodInches = dLeftPod * INCHES_PER_ENCODER_TICK;
        double dRightPodInches = dRightPod * INCHES_PER_ENCODER_TICK;
        double dMecanumPodInches = dMecanumPod * INCHES_PER_ENCODER_TICK;

        // so its easier to type
        double L = dLeftPodInches;
        double R = dRightPodInches;
        double M = dMecanumPodInches;
        double P = LR_ENCODER_DIST_FROM_CENTER;
        double Q = M_ENCODER_DIST_FROM_CENTER;

        // find robot relative deltas
        double dTheta = (L - R) / (2 * P);
        double dRobotX = M * sinXOverX(dTheta) + Q * Math.sin(dTheta) - L * cosXMinusOneOverX(dTheta) + P * (Math.cos(dTheta) - 1);
        double dRobotY = L * sinXOverX(dTheta) - P * Math.sin(dTheta) + M * cosXMinusOneOverX(dTheta) + Q * (Math.cos(dTheta) - 1);

        worldX += dRobotX * Math.cos(worldAngleRad) + dRobotY * Math.sin(worldAngleRad);
        worldY += dRobotY * Math.cos(worldAngleRad) - dRobotX * Math.sin(worldAngleRad);
        //worldAngleRad =  (leftPodNewPosition - rightPodNewPosition) * INCHES_PER_ENCODER_TICK / (2 * P);
        worldAngleRad += dTheta;
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double sinXOverX(double x){
        if (Math.abs(x) < 2){
            double retVal = 0;
            double top = 1;
            double bottom = 1;
            for (int i = 0; i < 9; i++){
                retVal += top/bottom;
                top *= -x*x;
                bottom *= (2 * i + 2)*(2 * i + 3);
            }
            return retVal;
        } else {
            return Math.sin(x)/x;
        }
    }

    /*
    taylor series expansion to make stuff COOL
     */
    private double cosXMinusOneOverX(double x){
        if (Math.abs(x) < 2){
            double retVal = 0;
            double top = -x;
            double bottom = 2;
            for (int i = 0; i < 9; i++){
                retVal += top/bottom;
                top *= -x*x;
                bottom *= (2 * i + 3)*(2 * i + 4);
            }
            return retVal;
        } else {
            return (Math.cos(x)-1)/x;
        }
    }

    public synchronized DcMotor getyLeft() {
        return yLeft;
    }

    public synchronized DcMotor getyRight() {
        return yRight;
    }

    public synchronized DcMotor getMecanum() {
        return mecanum;
    }

    public boolean isOn(){
        return isOn;
    }

    public String getName() {
        return "OdometryModule";
    }
}