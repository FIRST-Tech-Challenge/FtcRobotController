package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class GetRobotStatus extends Message {

    public GetRobotStatus() {
        super(MessageType.GET_ROBOT_STATUS);
    }
}
