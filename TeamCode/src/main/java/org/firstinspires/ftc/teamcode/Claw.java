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
    private boolean claw, joint, aDown, bDown;

    public Claw(HardwareMap hardwareMap){
        jointServo = hardwareMap.servo.get(JOINT_SERVO_NAME);
        clawServo = hardwareMap.servo.get(CLAW_SERVO_NAME);
        claw = false;
        joint = false;
        aDown = false;
        bDown = false;
    }

    public void update(Gamepad gamepad){
        boolean a = gamepad.a && !aDown;
        aDown = gamepad.a;
        boolean b = gamepad.b && !bDown;
        bDown = gamepad.b;

        if(a && claw){
            setJointPosition(0.75);
            claw = !claw;
        } else if (a && !claw){
            setJointPosition(0);
            claw = !claw;
        }

        if (gamepad.right_bumper && joint){
            setJointPosition(1);
            joint = !joint;
        } else if (gamepad.right_bumper && !joint){
            setJointPosition(0.25);
            joint = !joint;
        }
    }

    public void setJointPosition(double pos){ jointServo.setPosition(pos); }
    public void setClawPosition(double pos){ clawServo.setPosition(pos); }
}
