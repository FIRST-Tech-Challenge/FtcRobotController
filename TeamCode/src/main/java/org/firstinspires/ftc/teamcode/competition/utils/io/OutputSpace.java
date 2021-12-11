package org.firstinspires.ftc.teamcode.competition.utils.io;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.competition.utils.locations.ElevatorBottomLimitSwitchLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.HandDistanceSensorLocation;
import org.firstinspires.ftc.teamcode.competition.utils.locations.IntakeLimitSwitchLocation;

/**
 * This class can be used to receive output from locations. Locations get data from the interaction surfaces they handle, and then return that to this space. For example, if the location of touch sensor $y finds out $y is being touched, the location will send that data to the space on request. If something fails, it will consume any exceptions and act like nothing happened, returning an arbitrary value. When creating an OutputSpace, the OutputSpace attempts to create locations for every output location on the robot. For example, if the team decides to build a robot with 4 motors, locations for all 4 will be created and able to receive data from. The possible locations are defined inside this class. Since the OutputSpace is designed to handle output from the current robot, it should be built with the current robot and only the current robot in mind. Location classes are where all used locations should go. If they're unused on the current robot, leave them there. They should never be removed because they could be used later, and it's not like they really take up any extra resources anyway.
 */
public class OutputSpace {

    private final HardwareMap HARDWARE;
    private final HandDistanceSensorLocation HAND_DISTANCE_SENSOR;
    private final IntakeLimitSwitchLocation INTAKE_LIMIT_SWITCH;
    private final ElevatorBottomLimitSwitchLocation ELEVATOR_BOTTOM_LIMIT_SWITCH;

    public OutputSpace(HardwareMap hardware) {
        HARDWARE = hardware;
        HAND_DISTANCE_SENSOR = new HandDistanceSensorLocation(HARDWARE);
        INTAKE_LIMIT_SWITCH = new IntakeLimitSwitchLocation(HARDWARE);
        ELEVATOR_BOTTOM_LIMIT_SWITCH = new ElevatorBottomLimitSwitchLocation(HARDWARE);
    }

    public double receiveOutputFromHandDistanceSensor() {
        return HAND_DISTANCE_SENSOR.returnOutput();
    }

    public double receiveOutputFromIntakeLimitSwitch(IntakeLimitSwitchLocation.Values type) {
        return INTAKE_LIMIT_SWITCH.returnOutput(type);
    }

    public double receiveOutputFromElevatorBottomLimitSwitch(ElevatorBottomLimitSwitchLocation.Values type) {
        return ELEVATOR_BOTTOM_LIMIT_SWITCH.returnOutput(type);
    }

    public HardwareMap getHardware() {
        return HARDWARE;
    }

    public HandDistanceSensorLocation getHandDistanceSensor() {
        return HAND_DISTANCE_SENSOR;
    }

    public IntakeLimitSwitchLocation getIntakeLimitSwitch() {
        return INTAKE_LIMIT_SWITCH;
    }

    public ElevatorBottomLimitSwitchLocation getElevatorBottomLimitSwitch() {
        return ELEVATOR_BOTTOM_LIMIT_SWITCH;
    }
}
