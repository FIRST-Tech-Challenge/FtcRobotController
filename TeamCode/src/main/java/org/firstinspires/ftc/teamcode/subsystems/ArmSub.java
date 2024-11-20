package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants.ArmConstants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ArmSub extends SubsystemBase{
    Telemetry telemetry;

    public DcMotorEx armMotor;

    public ArmSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.telemetry = telemetry;

    }

    @Override
    public void periodic() {
        telemetry.addData("Arm Pos", armMotor.getCurrentPosition());
    }

    public DcMotor getMotor(){
        return armMotor;
    }

    public void armUp() {
        setPos(getMotor().getCurrentPosition() + 10, 1);
    }

    public void armDown() {
        setPos(getMotor().getCurrentPosition() - 10, 1);
    }

    public void armLow() {
        setPos(ArmConstants.armLow, -1);
    }

    public void armMed(){
        setPos(ArmConstants.armMed, -1);
    }

    public void armLowGoal() {
        setPos(ArmConstants.armLowGoal, -1);
    }

    public void armHighGoal() {
        setPos(ArmConstants.armHighGoal, -1);
    }

    public void setPos(int pos, int dir) {
        armMotor.setTargetPosition(pos);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (dir == 1) {
            armMotor.setVelocity(450);
        } else if (dir == -1) {
            armMotor.setVelocity(300);
        }
    }
}
