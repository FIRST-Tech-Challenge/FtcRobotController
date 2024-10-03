package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class Claw {
    private String JOINT_SERVO_NAME = "jointServo";
    private String CLAW_SERVO_NAME = "clawServo";

    private Servo jointServo;
    private Servo clawServo;

    public Claw(HardwareMap hardwareMap){
        jointServo = hardwareMap.servo.get(JOINT_SERVO_NAME);
        clawServo = hardwareMap.servo.get(CLAW_SERVO_NAME);
    }

    public void setJointPosition(double pos){ jointServo.setPosition(pos); }
    public void setClawPosition(double pos){ clawServo.setPosition(pos); }
}
