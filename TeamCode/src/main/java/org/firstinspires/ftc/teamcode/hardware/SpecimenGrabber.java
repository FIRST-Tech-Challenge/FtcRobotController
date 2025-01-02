package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
enum CurrentPostion {
    OPEN, CLOSED
}
public class SpecimenGrabber {

    private Servo grabber = null;
    public CurrentPostion currentPosition = CurrentPostion.OPEN;

    public void Init(HardwareMap hardwareMap) {
        grabber = hardwareMap.get(Servo.class, "specimangrabber");
    }
    public void Open() {
        grabber.setPosition(1.0);
    }

    public void Close() {
        grabber.setPosition(0.1);
    }

    public void processOpenClose() {
        if (currentPosition == CurrentPostion.CLOSED) {
            this.Open();
            currentPosition = CurrentPostion.OPEN;
        }
        else {
            this.Close();
            currentPosition = CurrentPostion.CLOSED;

        }
    }


}
