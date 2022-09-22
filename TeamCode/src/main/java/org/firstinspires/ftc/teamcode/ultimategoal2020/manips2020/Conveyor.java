package org.firstinspires.ftc.teamcode.ultimategoal2020.manips2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor implements EbotsManip2020 {

    private DcMotorEx conveyorMotor;


    public Conveyor(HardwareMap hardwareMap){
        conveyorMotor = hardwareMap.get(DcMotorEx.class, "conveyor");
        conveyorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        conveyorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Deprecated
    public void setPower(double targetPower){
        this.conveyorMotor.setPower(targetPower);
    }

    @Override
    public void handleGamepadInput(Gamepad gamepad) {
        double inputThreshold = 0.3;
        double conveyorInput = -gamepad.left_stick_y;

        // Condition the input signal to either be -1, 0, or 1
        double conveyorPower = (Math.abs(conveyorInput) < inputThreshold) ? 0 : Math.signum(conveyorInput) * 1;

        conveyorMotor.setPower(conveyorPower);

    }

    @Override
    public void stop() {
        conveyorMotor.setPower(0);
    }

    public void startConveyor(){
        conveyorMotor.setPower(1.0);
    }

}
