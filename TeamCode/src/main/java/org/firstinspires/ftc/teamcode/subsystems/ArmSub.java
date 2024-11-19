package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class ArmSub extends SubsystemBase{
    Telemetry telemetry;

    public DcMotorEx armMotor;

    public ArmSub(HardwareMap hardwareMap, Telemetry telemetry) {
        this.armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        this.armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        setPos((int) (getMotor().getCurrentPosition() + ((0.0006*Math.pow(getMotor().getCurrentPosition()-640, 2))+16.5)), 1);
    }
    public void armDown() {
        setPos((int) 10, -1);
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
