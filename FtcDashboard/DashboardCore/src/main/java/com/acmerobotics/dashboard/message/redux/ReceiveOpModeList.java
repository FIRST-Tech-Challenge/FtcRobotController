package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;
import java.util.List;

public class ReceiveOpModeList extends Message {
    private List<String> opModeList;

    public ReceiveOpModeList(List<String> opModeList) {
        super(MessageType.RECEIVE_OP_MODE_LIST);

        this.opModeList = opModeList;
    }
}
