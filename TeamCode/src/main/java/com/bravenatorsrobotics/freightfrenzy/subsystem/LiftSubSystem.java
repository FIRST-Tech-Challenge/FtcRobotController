package com.bravenatorsrobotics.freightfrenzy.subsystem;

import com.bravenatorsrobotics.common.operation.OperationMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class LiftSubSystem extends AbstractSubSystem {

    private final DcMotorEx liftMotor;

    public LiftSubSystem(OperationMode<?> operationMode) {
        super(operationMode);

        liftMotor = operationMode.hardwareMap.get(DcMotorEx.class, "lift");
    }

}
