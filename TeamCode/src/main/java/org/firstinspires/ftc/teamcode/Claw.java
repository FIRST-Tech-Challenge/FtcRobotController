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
    private boolean claw, joint;

    public Claw(HardwareMap hardwareMap){
        jointServo = hardwareMap.servo.get(JOINT_SERVO_NAME);
        clawServo = hardwareMap.servo.get(CLAW_SERVO_NAME);
        claw = false;
        joint = false;
    }

    public void update(Gamepad gamepad){
        if(gamepad.a && claw){
            setJointPosition(0.75);
            claw = !claw;
        } else if (gamepad.a && !claw){
            setJointPosition(0);
            claw = !claw;
        }

        if (gamepad.b && joint){
            setJointPosition(0.75);
            joint = !joint;
        } else if (gamepad.b && !joint){
            setJointPosition(0);
            joint = !joint;
        }
    }

    public void setJointPosition(double pos){ jointServo.setPosition(pos); }
    public void setClawPosition(double pos){ clawServo.setPosition(pos); }
}
