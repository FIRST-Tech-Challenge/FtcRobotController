package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot {
    private final boolean debug;

    private Pos position;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    private Drive drive;

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
    Module leftModule;
    Module rightModule;

    // Declared IMU
    BNO055IMU imu;

    private DcMotor getMotor(String id) {
        return hardwareMap.dcMotor.get(id);
    }

    private Servo getServo(String id) {
        return hardwareMap.servo.get(id);
    }

    public Robot(boolean debug, Pos startingPos, Telemetry telemetry, HardwareMap hardwareMap) {
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
        leftModule = new Module(Drive.ModuleSide.LEFT, getMotor("topLeft"), getMotor("bottomLeft"), telemetry);
        leftModule = new Module(Drive.ModuleSide.RIGHT, getMotor("topRight"), getMotor("bottomRight"), telemetry);

        // IMU setting
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        this.drive = new Drive(this.debug, position, leftModule, rightModule, imu, telemetry);
    }
}
