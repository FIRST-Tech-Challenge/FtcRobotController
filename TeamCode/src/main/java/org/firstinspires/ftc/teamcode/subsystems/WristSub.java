package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class WristSub extends SubsystemBase {
    Telemetry telemetry;

    public CRServo wrist;

    public WristSub(HardwareMap hardwareMap, Telemetry tm) {
        wrist = hardwareMap.get(CRServo.class, "wrist");
        this.telemetry = tm;
    }

    @Override
    public void periodic() {

    }

    public CRServo getServo(){
        return wrist;
    }

    /**
    * Sets the speed of the wrist.
    *
    * @param speed the speed to set the wrist to
     */
    public void setSpeed(double speed) {
        telemetry.addData("Wrist called with speed of " + speed);
        wrist.setPower(speed);
    }
}