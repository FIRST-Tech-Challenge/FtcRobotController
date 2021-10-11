package org.firstinspires.ftc.teamcode.core.hardware.attachment.impl;

import org.firstinspires.ftc.teamcode.core.hardware.attachment.api.Attachment;

public class BasicSingleMotorAttachment implements Attachment {
    private final AttachmentProfile profile;

    public BasicSingleMotorAttachment(AttachmentProfile profile) {
        this.profile = profile;
    }

    @Override
    public void move(double position, double speed) {

    }

    @Override
    public void move(double position) {

    }
}
