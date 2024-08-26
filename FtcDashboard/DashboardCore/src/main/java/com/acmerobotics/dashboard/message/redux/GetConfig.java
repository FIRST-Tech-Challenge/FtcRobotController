package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class GetConfig extends Message {
    public GetConfig() {
        super(MessageType.GET_CONFIG);
    }
}
