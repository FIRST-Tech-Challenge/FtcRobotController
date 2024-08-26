package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class ReceiveImage extends Message {
    private String imageString;

    public ReceiveImage(String imageString) {
        super(MessageType.RECEIVE_IMAGE);

        this.imageString = imageString;
    }
}
