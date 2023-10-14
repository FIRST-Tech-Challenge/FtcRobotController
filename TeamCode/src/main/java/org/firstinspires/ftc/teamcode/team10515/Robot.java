package org.firstinspires.ftc.teamcode.team10515;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.drivers.RevCRServo;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.control.EnhancedGamepad;

public abstract class Robot extends OpMode {
    private static final boolean isUsingComputer = false;
    private static final boolean optimizeInputChange = false;

    private static final double WHEEL_DIAMETER = 100d / 25.4d; //in
    private static final double L1 = 9d; //in
    private static final double L2 = 9d; //in
    private static final double D1 = 9d; //in
    private static final double D2 = 9d; //in
    private static final double ROBOT_LENGTH = getL1() + getL2(); //in

    private static volatile Pose2d drivetrainPower;
    private static MecanumDriveModel driveModel;
    private static MecanumDriveMPC driveMPC;
    private static MecanumRunnableLQR runnableLQR;
    private static SimpleMatrix state;
    private static SimpleMatrix lastInput;

    private static EnhancedGamepad enhancedGamepad1;
    private static EnhancedGamepad enhancedGamepad2;
    private RevMotor[]   motors;
    private RevServo[]   servos;
    private RevCRServo[] crServos;

    private static TimeProfiler updateRuntime;
    private static double dt;

    @Override
    public void init() {
        setDrivetrainPower(Pose2d.identity());
        setEnhancedGamepad1(new EnhancedGamepad(gamepad1));
        setEnhancedGamepad2(new EnhancedGamepad(gamepad2));
        setUpdateRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
    }

    @Override
    public void start() {
        getUpdateRuntime().start();
    }

    @Override
    public void loop() {
        setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));
        getEnhancedGamepad1().update();
        getEnhancedGamepad2().update();
    }

    public static EnhancedGamepad getEnhancedGamepad1() {
        return enhancedGamepad1;
    }

    public static void setEnhancedGamepad1(EnhancedGamepad enhancedGamepad1) {
        Robot.enhancedGamepad1 = enhancedGamepad1;
    }

    public static EnhancedGamepad getEnhancedGamepad2() {
        return enhancedGamepad2;
    }

    public static void setEnhancedGamepad2(EnhancedGamepad enhancedGamepad2) {
        Robot.enhancedGamepad2 = enhancedGamepad2;
    }

    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updateRuntime) {
        Robot.updateRuntime = updateRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double dt) {
        Robot.dt = dt;
    }

    public static Pose2d getDrivetrainPower() {
        return drivetrainPower;
    }

    public static void setDrivetrainPower(Pose2d drivetrainPower) {
        Robot.drivetrainPower = drivetrainPower;
    }

    public static double getL1() {
        return L1;
    }

    public static double getL2() {
        return L2;
    }

    public static double getD1() {
        return D1;
    }

    public static double getD2() {
        return D2;
    }

    public static double getRobotLength() {
        return ROBOT_LENGTH;
    }

    public static double getWheelDiameter() {
        return WHEEL_DIAMETER;
    }

    public static double getWheelRadius() {
        return getWheelDiameter() / 2d;
    }

    public static boolean isUsingComputer() {
        return isUsingComputer;
    }

    public static boolean isOptimizeInputChange() {
        return optimizeInputChange;
    }

    public static MecanumDriveModel getDriveModel() {
        return driveModel;
    }

    public static void setDriveModel(MecanumDriveModel driveModel) {
        Robot.driveModel = driveModel;
    }

    public static MecanumDriveMPC getDriveMPC() {
        return driveMPC;
    }

    public static void setDriveMPC(MecanumDriveMPC driveMPC) {
        Robot.driveMPC = driveMPC;
    }

    public static MecanumRunnableLQR getRunnableLQR() {
        return runnableLQR;
    }

    public static void setRunnableLQR(MecanumRunnableLQR runnableLQR) {
        Robot.runnableLQR = runnableLQR;
    }

    public static SimpleMatrix getState() {
        return state;
    }

    public static void setState(SimpleMatrix state) {
        Robot.state = state;
    }

    public static SimpleMatrix getLastInput() {
        return lastInput;
    }

    public static void setLastInput(SimpleMatrix lastInput) {
        Robot.lastInput = lastInput;
    }

    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public RevServo[] getServos() {
        return servos;
    }

    public void setServos(RevServo[] servos) {
        this.servos = servos;
    }

    public RevCRServo[] getCrServos() {
        return crServos;
    }

    public void setCrServos(RevCRServo[] crServos) {
        this.crServos = crServos;
    }
}
