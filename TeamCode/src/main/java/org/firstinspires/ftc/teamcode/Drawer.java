package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drawer {
    private String DRAWER_MOTOR_NAME = "drawerMotor";
    private DcMotor drawerMotor = null;
    private Claw claw;
    
    public Drawer(HardwareMap hardwareMap) {
        drawerMotor = hardwareMap.dcMotor.get(DRAWER_MOTOR_NAME);
        drawerMotor.setDirection(DcMotor.Direction.REVERSE);
        claw = new Claw(hardwareMap);
    }

    public void update(Gamepad gamepad){
        update((gamepad.right_trigger - gamepad.left_trigger));
        claw.update(gamepad);
    }

    public void update(double power) {
        drawerMotor.setPower(power);
    }

    public void setJointPosition(double pos){ claw.setJointPosition(pos); }
    public void setClawPosition(double pos){ claw.setClawPosition(pos); }

    }
    
     
   
