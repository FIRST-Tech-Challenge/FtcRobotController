package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.RobotStatus;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class ReceiveRobotStatus extends Message {
    private RobotStatus status;

    public ReceiveRobotStatus(RobotStatus robotStatus) {
        super(MessageType.RECEIVE_ROBOT_STATUS);

        status = robotStatus;
    }
}
