package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.HashMap;

public class InstructionHandler {
    private int numberOfCurrentInstructions = 0;
    private HashMap<Integer, SystemState> instructionSet = new HashMap<>();
    public int addInstruction(double ignorePosTime, SparkFunOTOS.Pose2D drivePose, armPose armPosition, wristState wristPosition, clawState clawPosition) {
        SystemState state = new SystemState();
        state.ignorePosTime = ignorePosTime;
        state.drivePose = drivePose;
        state.armPosition = armPosition;
        state.wristPosition = wristPosition;
        state.clawPosition = clawPosition;
        instructionSet.put(numberOfCurrentInstructions, state);
        numberOfCurrentInstructions = numberOfCurrentInstructions + 1;
        return numberOfCurrentInstructions - 1;
    }
    public void reuseInstruction(int instructionNumber) {
        instructionSet.put(numberOfCurrentInstructions, instructionSet.get(instructionNumber));
        numberOfCurrentInstructions = numberOfCurrentInstructions + 1;
    }
    public HashMap<Integer, SystemState> getInstructions() {
        return instructionSet;
    }
}
