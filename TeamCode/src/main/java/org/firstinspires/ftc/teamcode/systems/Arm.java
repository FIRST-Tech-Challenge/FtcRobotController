package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm extends System{

    private double casting = 0.0, increase = 1.0;
    private double inc = 1440.0;
    private int increment = 1440;

    private DcMotor armMotor;
    private int pos, initPos;
    private int cnt;

    public Arm(HardwareMap hw, Controller controller){
        super(hw, controller);
    }

    @Override
    public void init() {
        // get motor
        armMotor = hw.dcMotor.get("outputController");
        armMotor.setTargetPosition(0);
        casting = inc * increase;
        increment = (int) inc;
        pos = 0;
        initPos = armMotor.getCurrentPosition();
    }

    @Override
    public void update() {
        if(controller.gamepad2.dpad_down){
            setPosThree();
        }else if(controller.gamepad2.dpad_right){
            setPosTwo();
        }else if(controller.gamepad2.dpad_down){
            setPosOne();
        }
        controller.telemetry.addData("Arm Position: ", pos);
    }

    private void setPosOne(){
        pos = 0;
        updatePos();
    }

    private void setPosTwo(){
        pos = increment;
        updatePos();
    }

    private void setPosThree(){
        pos = increment * 2;
        updatePos();
    }

    private void updatePos(){
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setTargetPosition(pos);
        if(initPos > pos) armMotor.setPower(-1);
        else armMotor.setPower(1);
        initPos = pos;
    }
}
