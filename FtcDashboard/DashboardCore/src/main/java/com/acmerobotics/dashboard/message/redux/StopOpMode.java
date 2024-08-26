package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class StopOpMode extends Message {
    public StopOpMode() {
        super(MessageType.STOP_OP_MODE);
    }
}
