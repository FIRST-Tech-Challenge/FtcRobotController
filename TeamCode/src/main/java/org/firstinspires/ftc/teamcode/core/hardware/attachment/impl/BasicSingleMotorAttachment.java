package org.firstinspires.ftc.teamcode.core.hardware.attachment.impl;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.hardware.attachment.api.Attachment;

public class BasicSingleMotorAttachment implements Attachment {
    private final DcMotor motor;
    private final AttachmentProfile profile;

    public BasicSingleMotorAttachment(DcMotor motor, AttachmentProfile profile) {
        this.motor = motor;
        this.profile = profile;

    }

    @Override
    public void moveOnThisThread(double position, double speed) {

    }

    @Override
    public void move(double position, double speed) {
        
    }

    @Override
    public void move(double position) {
        this.move(position, this.profile.maxPower);
    }
}
