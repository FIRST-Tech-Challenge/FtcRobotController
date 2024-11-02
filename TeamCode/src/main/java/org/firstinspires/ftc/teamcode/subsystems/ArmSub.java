package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ArmSub extends SubsystemBase{
    Telemetry telemetry;

    public DcMotor armMotor;

    public ArmSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;

    }

    @Override
    public void periodic() {
        telemetry.addData("Arm Rotation", armMotor.getCurrentPosition());
    }

    public DcMotor getMotor(){
        return armMotor;
    }

    public void move(double speed) {
        armMotor.setPower(speed);
    }
    public void setPos(int pos) {
        armMotor.setTargetPosition(pos);
    }
}
