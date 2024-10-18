package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

public class RobotHardware {

    public FtcDashboard dashboard;
    public HardwareMap hardwareMap;
    public DcMotorEx elevatorMotor;
    public DcMotorEx wormGearMotor;
    public Servo wristServo;
    public TouchSensor elevatorLimitSwitch;

    /**
     * @param newDashboard the FtcDashboard instance for adding telemetry and the such
     * @param newHardwareMap the hardware map for the robot
     */
    public RobotHardware(FtcDashboard newDashboard, HardwareMap newHardwareMap) {
        dashboard = newDashboard;
        hardwareMap = newHardwareMap;
        elevatorMotor = hardwareMap.get(DcMotorEx.class, "ElevatorMotor");
        wormGearMotor = hardwareMap.get(DcMotorEx.class, "WormGearMotor");
        wristServo = hardwareMap.get(Servo.class, "WristServo");
        elevatorLimitSwitch = hardwareMap.get(TouchSensor.class, "ElevatorLimitSwitch");
    }
}
