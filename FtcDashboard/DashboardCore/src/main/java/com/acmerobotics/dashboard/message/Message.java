package com.acmerobotics.dashboard.message;

/**
 * Class for representing dashboard messages.
 */
public class Message {
    private MessageType type;

    /**
     * Creates a message.
     *
     * @param type message type
     */
    public Message(MessageType type) {
        this.type = type;
    }

    /**
     * Returns the message type.
     */
    public MessageType getType() {
        return type;
    }
}
