package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drawer {
    private String DRAWER_MOTOR_NAME = "drawerMotor";
    //private String RIGHT_DRAWER_MOTOR_NAME = "rightDrawerMotor";
    //private DcMotor leftDrawerMotor = null;
    private DcMotor drawerMotor = null;
    private Claw claw;
    
    public Drawer(HardwareMap hardwareMap) {
        //leftDrawerMotor = hardwareMap.dcMotor.get(LEFT_DRAWER_MOTOR_NAME);
        drawerMotor = hardwareMap.dcMotor.get(DRAWER_MOTOR_NAME);
        //leftDrawerMotor.setDirection(DcMotor.Direction.FORWARD);
        drawerMotor.setDirection(DcMotor.Direction.FORWARD);
        claw = new Claw(hardwareMap);
    }

    public void update(Gamepad gamepad){
        update((gamepad.right_trigger - gamepad.left_trigger) * 0.1);
        claw.update(gamepad);
    }

    public void update(double power) {
        //leftDrawerMotor.setPower(power);
        drawerMotor.setPower(power);
    }

    public void setJointPosition(double pos){ claw.setJointPosition(pos); }
    public void setClawPosition(double pos){ claw.setClawPosition(pos); }

    }
    
     
   
