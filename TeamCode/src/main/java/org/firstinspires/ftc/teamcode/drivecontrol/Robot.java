package org.firstinspires.ftc.teamcode.drivecontrol;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private final boolean debug;

    private Position position;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private DriveController driveController;

    // Declared Motors
    DcMotor arm;
    DcMotor lift;
    DcMotor flywheel;
    DcMotor intake;

    // Declared Servos
    Servo pusher;
    Servo lifter;
    Servo claw;

    // Declared Modules
    WheelModule leftWheelModule;
    WheelModule rightWheelModule;

    // Declared IMU
    BNO055IMU imu;

    private DcMotor getMotor(String id) {
        return hardwareMap.dcMotor.get(id);
    }

    private Servo getServo(String id) {
        return hardwareMap.servo.get(id);
    }

    public Robot(boolean debug, Position startingPos, Telemetry telemetry, HardwareMap hardwareMap) {
        this.debug = debug;

        this.position = startingPos;
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        // Motor setting
        arm = getMotor("arm");
        lift = getMotor("lift");
        flywheel = getMotor("flywheel");
        intake = getMotor("intake");

        // Servo setting
        pusher = getServo("pusher");
        lifter = getServo("lifter");
        claw = getServo("claw");

        // Module setting
        leftWheelModule = new WheelModule(DriveController.ModuleSide.LEFT, getMotor("topLeft"), getMotor("bottomLeft"), telemetry);
        leftWheelModule = new WheelModule(DriveController.ModuleSide.RIGHT, getMotor("topRight"), getMotor("bottomRight"), telemetry);

        // IMU setting
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.driveController = new DriveController(this.debug, position, leftWheelModule, rightWheelModule, imu, telemetry);
    }

    public void updateDrivePositionTracking() {
        driveController.updatePositionTracking();
    }

    public void updateUsingJoysticks(Vector2D j1, Vector2D j2) {
        driveController.updateUsingJoysticks(j1, j2);
    }

    public void initIMU() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu.initialize(parameters);
    }
}
