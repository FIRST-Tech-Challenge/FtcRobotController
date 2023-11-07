package org.firstinspires.ftc.teamcode.Commands;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PID;

public class Arm extends Command {
    private DcMotor Arm1;
    double targetPosition;
    org.firstinspires.ftc.teamcode.PID PID = new PID(0.1, 0, 0);
    public Arm(HardwareMap hardwareMap, double targetPosition){
        Arm1 = hardwareMap.dcMotor.get("Arm1");
        this.targetPosition = targetPosition;
    }
    public void start(){
        Arm1.setPower(1);
        PID.setSetPoint(targetPosition);
        Arm1.setTargetPosition(PID.getSetPoint());
        Arm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void execute(){
        PID.updatePID(Arm1.getCurrentPosition());
        Arm1.setTargetPosition(PID.getSetPoint());
        Arm1.setPower(1);
    }
    public void end(){
        Arm1.setPower(0);
    }

    public boolean isFinished() {
        if (Arm1.getCurrentPosition() >= targetPosition + 15||  Arm1.getCurrentPosition() >= targetPosition - 15) {
            return true;
        }
        return false;
    }
}
