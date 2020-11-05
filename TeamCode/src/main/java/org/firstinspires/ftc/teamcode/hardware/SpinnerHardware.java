package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.playmaker.RobotHardware;
import org.firstinspires.ftc.teamcode.util.OmniDrive;

public abstract class SpinnerHardware extends RobotHardware {

    public DcMotor spinnerLeft;
    public DcMotor spinnerRight;
    public DcMotor chainLift;

    @Override
    public void initializeHardware() {
        spinnerLeft = this.initializeDevice(DcMotor.class, "spinnerLeft");
        spinnerLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinnerRight = this.initializeDevice(DcMotor.class, "spinnerRight");
        spinnerRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        chainLift = this.initializeDevice(DcMotor.class, "chain");
    }

}