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

    private boolean isAPressed;
    private boolean isXPressed;


    public Claw(HardwareMap hardwareMap) {
        jointServo = hardwareMap.servo.get(JOINT_SERVO_NAME);
        clawServo = hardwareMap.servo.get(CLAW_SERVO_NAME);
        isAPressed = false;
        isXPressed = false;
    }

    public void update(Gamepad gamepad) {
        //claw and servo are switched in wiring, and I'm too lazy to fix
        /*
        if(gamepad.x) {
            if (clawServo.getPosition() > 0.5){
                setClawPosition(0);
            } else {
                setClawPosition(0.80);
            }
            isAPressed = !isAPressed;
        }
         */
        if(gamepad.x) {
            setClawPosition(0);
        }

        if (gamepad.y){
            setClawPosition(0.8);
        }
        if (gamepad.a){
            setJointPosition(0);
        }
        if(gamepad.b){
            setJointPosition(0.26);
        }

        /*
        if(gamepad.a) {
            if (jointServo.getPosition() > 0.125){
                setJointPosition(0);
            } else {
                setJointPosition(0.26);
            }
            isXPressed = !isXPressed;
        }
         */
    }

    public void setJointPosition(double pos){ jointServo.setPosition(pos); }
    public void setClawPosition(double pos){ clawServo.setPosition(pos); }
}
