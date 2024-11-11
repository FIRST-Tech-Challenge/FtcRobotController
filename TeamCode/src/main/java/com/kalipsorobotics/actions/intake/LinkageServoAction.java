package com.kalipsorobotics.actions.intake;
import com.qualcomm.robotcore.hardware.Servo;
import com.kalipsorobotics.modules.Intake;


public class LinkageServoAction {
    Intake intake;

    Servo linkageServo;

    public LinkageServoAction(Intake intake) {
        this.intake = intake;
        linkageServo = intake.getLinkageServo1();
    }

    public void goToPosition(double position) {
        linkageServo.setPosition(position);
    }

}
