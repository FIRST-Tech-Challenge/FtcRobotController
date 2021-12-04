package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CVRec.GameElement;
import org.firstinspires.ftc.teamcode.autonomous.AutoDot;
import org.firstinspires.ftc.teamcode.calibration.BotCalibConfig;
import org.firstinspires.ftc.teamcode.calibration.MotorReductionBot;
import org.firstinspires.ftc.teamcode.odometry.IBaseOdometry;

import java.io.File;

public interface IOdoBot {
    void init(LinearOpMode owner, HardwareMap hw, Telemetry t) throws Exception;
    void initGyro();
    void initCalibData() throws Exception;
    BotCalibConfig getCalibConfig();
    double getLeftOdometer();
    double getRightOdometer();
    double getLeftBackOdometer();
    double getRightBackOdometer();
    Telemetry getTelemetry();
    void curveTo(BotMoveProfile profile, IBaseOdometry locator);
    void moveToPos(BotMoveProfile profile, IBaseOdometry locator);
    void spin(BotMoveProfile profile, IBaseOdometry locator);
    void spinCalib(double degrees, double speed, IBaseOdometry locator);
    double getEncoderCountsPerInch();
    double getHorizontalOdometer();
    double getRobotCenterX();
    double getRobotCenterY();
    void strafeRight(double speed);
    void strafeLeft(double speed);
    void move(double drive, double turn);
    void moveTo(BotMoveProfile profile);
    void diagTo(BotMoveProfile profile, IBaseOdometry locator);
    void initDetectorThread(String side, LinearOpMode caller);
    void stopDetection();
    double strafeToCalib(double speed, double inches, boolean left, MotorReductionBot calib, IBaseOdometry locator);
    void addNamedCoordinate(AutoDot dot);
    double getGyroHeading();
    RobotMovementStats moveToCalib(double leftspeed, double rightspeed, double inches, MotorReductionBot mr, double breakPoint, IBaseOdometry locator);
    File getCalibConfigFile();
    void turnLeft(double speed, boolean forward);
    void turnRight(double speed, boolean forward);
    void stop();
    void spinLeft(double speed, boolean forward);
    void spinRight(double speed, boolean forward);
    void diagToCalib(double speed, double lowSpeed, double diagInches, boolean leftAxis, MotorReductionBot calib, IBaseOdometry locator);
    void reverseEncoderDirection();
    int getEncoderDirection();
    AutoDot getDetectionResult();
}
