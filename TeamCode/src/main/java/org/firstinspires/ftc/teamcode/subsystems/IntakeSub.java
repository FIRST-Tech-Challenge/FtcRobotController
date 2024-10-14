package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSub extends SubsystemBase {
    Telemetry telemetry;

    public CRServo intake;

    public IntakeSub(HardwareMap hardwareMap, Telemetry tm) {
        intake = hardwareMap.get(CRServo.class, "intake");
        this.telemetry = tm;
    }

    @Override
    public void periodic() {

    }

    public CRServo getServo(){
        return intake;
    }

    /**
    * Sets the speed of the intake.
    *
    * @param speed the speed to set the intake to
     */
    public void setSpeed(double speed) {
        telemetry.addData("Intake called with speed of ", speed);
        intake.setPower(speed);
    }
}