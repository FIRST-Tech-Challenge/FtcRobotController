package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Arm{
    DcMotor armMotor= (DcMotor) hardwareMap.get("armMotor");

    public void setArmMotor(DcMotor armMotor) {
        this.armMotor = armMotor;
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));

        


    }
}
