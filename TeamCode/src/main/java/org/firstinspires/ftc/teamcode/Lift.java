package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {

    public final int liftEncoderMin = 1200; // Set your minimum encoder value here
    private final int liftEncoderMax = 3500;
    public DcMotor liftMotor;
    Gamepad gamepad2;


    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        liftMotor = hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gamepad2 = gamepad2;
    }

    public void update(){
        setLiftPowerBasedOnGamepad(gamepad2);
    }

    private void setLiftPowerBasedOnGamepad(Gamepad gamepad){
        double power = -gamepad.right_stick_y;
        // Set your maximum encoder value here
        if ((power > 0 && liftMotor.getCurrentPosition() < liftEncoderMax) ||
                (power < 0 && liftMotor.getCurrentPosition() > liftEncoderMin)) {
            liftMotor.setPower(power);
        } else {
            liftMotor.setPower(0);
        }
    }
}
