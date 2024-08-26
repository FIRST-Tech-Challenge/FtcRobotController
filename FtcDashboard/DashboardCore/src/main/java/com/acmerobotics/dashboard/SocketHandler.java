package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.message.Message;

public interface SocketHandler {
    void onOpen();

    void onClose();

    // Returns true iff the message was handled
    boolean onMessage(Message message);
}
