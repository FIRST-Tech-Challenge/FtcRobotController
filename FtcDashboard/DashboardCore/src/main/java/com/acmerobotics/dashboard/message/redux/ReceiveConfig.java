package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class ReceiveConfig extends Message {
    private CustomVariable configRoot;

    public ReceiveConfig(CustomVariable configRoot) {
        super(MessageType.RECEIVE_CONFIG);

        this.configRoot = configRoot;
    }
}
