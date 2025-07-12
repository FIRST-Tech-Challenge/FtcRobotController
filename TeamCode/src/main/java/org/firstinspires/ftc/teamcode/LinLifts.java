package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LinLifts {
    DcMotor lift = null;
    double targetPosition = 0;
    double kP = .001;


    public void init(HardwareMap HwMap) {
        lift = HwMap.get(DcMotor.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void updateLift () {
        double error = targetPosition - lift.getCurrentPosition();

        double power = error * kP;
        lift.setPower(power);
    }
    void setLift (double target) {
        targetPosition = target;
    }
}
