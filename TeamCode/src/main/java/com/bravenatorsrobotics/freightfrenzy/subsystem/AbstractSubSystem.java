package com.bravenatorsrobotics.freightfrenzy.subsystem;

import com.bravenatorsrobotics.common.operation.OperationMode;

public abstract class AbstractSubSystem {

    protected final OperationMode<?> operationMode;

    public AbstractSubSystem(OperationMode<?> operationMode) {
        this.operationMode = operationMode;
    }

}