package com.kalipsorobotics.actions.intake;
import com.kalipsorobotics.utilities.KServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.kalipsorobotics.modules.Intake;


public class LinkageServoAction {
    Intake intake;

    KServo linkageServo;

    public LinkageServoAction(Intake intake) {
        this.intake = intake;
        linkageServo = intake.getLinkageServo1();
    }

    public void goToPosition(double position) {
        linkageServo.setPosition(position);
    }

}
