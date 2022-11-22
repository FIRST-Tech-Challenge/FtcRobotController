package org.firstinspires.ftc.teamcode.robotParts;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public abstract class RobotPart {

    protected Telemetry.Item telemetry;

    protected Map<String, DcMotorEx> motors;
    protected Map<String, Servo> servos;
    protected Map<String, CRServo> crServos;


}
