package com.acmerobotics.dashboard.message.redux;

import com.acmerobotics.dashboard.config.variable.CustomVariable;
import com.acmerobotics.dashboard.message.Message;
import com.acmerobotics.dashboard.message.MessageType;

public class SaveConfig extends Message {
    private CustomVariable configDiff;

    public SaveConfig(CustomVariable configDiff) {
        super(MessageType.SAVE_CONFIG);

        this.configDiff = configDiff;
    }

    public CustomVariable getConfigDiff() {
        return configDiff;
    }
}
