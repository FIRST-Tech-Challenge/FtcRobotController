package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public final DcMotor liftMotor;
    public int liftEncoderMin = 0; // Set your minimum encoder value here
    private int liftEncoderMax = 0; // Set your maximum encoder value here
    Gamepad gamepad2;


    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        this.gamepad2 = gamepad2;
    }

    public void update(){
        setLiftPowerBasedOnGamepad(gamepad2);
    }

    private void setLiftPowerBasedOnGamepad(Gamepad gamepad){
        double power = gamepad.right_stick_y;
        if ((power > 0 && liftMotor.getCurrentPosition() < liftEncoderMax) ||
                (power < 0 && liftMotor.getCurrentPosition() > liftEncoderMin)) {
            liftMotor.setPower(power);
        } else {
            liftMotor.setPower(0);
        }
    }
}
