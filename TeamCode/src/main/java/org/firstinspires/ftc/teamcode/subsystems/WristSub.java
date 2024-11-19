package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.WristConstants;
public class WristSub extends SubsystemBase {
    Telemetry telemetry;

    public Servo wrist;

    public WristSub(HardwareMap hardwareMap, Telemetry tm) {
        wrist = hardwareMap.get(Servo.class, "wrist");
        this.telemetry = tm;
    }

    @Override
    public void periodic() {

    }

    public Servo getServo(){
        return wrist;
    }

    /**
    * Sets the position of the wrist.
    *
    * @param position the angle of the wrist
     */
    public void setPosition(double position) {
        telemetry.addData("Wrist set to", position);
        wrist.setPosition(position);
    }

   public double getPosition () {
        return wrist.getPosition();
   }

}