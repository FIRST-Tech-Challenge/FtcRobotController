package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drawer {
    private String LEFT_DRAWER_MOTOR_NAME = "leftDrawerMotor";
    private String RIGHT_DRAWER_MOTOR_NAME = "rightDrawerMotor";
    private DcMotor leftDrawerMotor = null;
    private DcMotor rightDrawerMotor = null;
    
    public Drawer(HardwareMap hardwareMap) {
        leftDrawerMotor = hardwareMap.dcMotor.get(LEFT_DRAWER_MOTOR_NAME);
        rightDrawerMotor = hardwareMap.dcMotor.get(RIGHT_DRAWER_MOTOR_NAME);
        leftDrawerMotor.setDirection(DcMotor.Direction.FORWARD);
        rightDrawerMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void update(Gamepad gamepad){
        update((gamepad.right_trigger - gamepad.left_trigger) * 0.1);
    }

    public void update(double power) {
        leftDrawerMotor.setPower(power);
        rightDrawerMotor.setPower(power);
        
    }
    public void update1(double power) {
        leftDrawerMotor.setPower(power);
        rightDrawerMotor.setPower(power);
    }

    }
    
     
   
