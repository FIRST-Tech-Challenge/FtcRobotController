package org.firstinspires.ftc.teamcode.competition.utils.locations;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.InteractionSurface;
import org.firstinspires.ftc.teamcode.competition.utils.interactions.items.StandardMotor;

import java.util.ArrayList;

public class ElevatorRightLiftMotorLocation extends Location {

    private final StandardMotor RIGHT;

    public ElevatorRightLiftMotorLocation(HardwareMap hardware) {
        RIGHT = new StandardMotor(hardware, hardware.appContext.getString(R.string.LIFT_RIGHT_ELEVATOR_MOTOR), DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void stop() {
        RIGHT.stop();
    }

    @Override
    public boolean isInputLocation() {
        return true;
    }

    @Override
    public boolean isOutputLocation() {
        return false;
    }

    @Override
    public InteractionSurface getInternalInteractionSurface() {
        return RIGHT;
    }

}
