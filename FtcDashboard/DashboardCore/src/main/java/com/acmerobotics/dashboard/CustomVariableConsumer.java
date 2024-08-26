package com.acmerobotics.dashboard;

import com.acmerobotics.dashboard.config.variable.CustomVariable;

public interface CustomVariableConsumer {
    void accept(CustomVariable customVariable);
}
