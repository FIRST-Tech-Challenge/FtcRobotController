package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class StartOpMode extends Message {
    public StartOpMode() {
        super(MessageType.START_OP_MODE);
    }
}
