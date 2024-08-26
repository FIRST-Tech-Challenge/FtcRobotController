package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class InitOpMode extends Message {
    private String opModeName;

    public InitOpMode(String opModeName) {
        super(MessageType.INIT_OP_MODE);

        this.opModeName = opModeName;
    }

    public String getOpModeName() {
        return opModeName;
    }
}
