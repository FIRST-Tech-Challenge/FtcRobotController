package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends System {

    private DcMotor inputSpinner;

    public Intake(HardwareMap hw, Controller controller){
        super(hw, controller);
    }

    @Override
    public void init() {
        // get the motor
        inputSpinner = hw.dcMotor.get("inputSpinner");
        inputSpinner.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void update() {

        if (controller.gamepad2.a){
            inputSpinner.setPower(1);
        }else{
            inputSpinner.setPower(0);
        }

    }

}
