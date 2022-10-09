package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BrainStemRobot {
    private OpMode opMode;
    private Telemetry telemetry;
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry, LinearOpMode opMode){
        this.telemetry = telemetry;
        this.opMode = opMode;
        frontLeft = (DcMotorEx)hwMap.dcMotor.get("FL");
        frontRight = (DcMotorEx)hwMap.dcMotor.get("FR");
        backRight = (DcMotorEx) hwMap.dcMotor.get("BR");
        backLeft = (DcMotorEx) hwMap.dcMotor.get("BL");
    }
}
