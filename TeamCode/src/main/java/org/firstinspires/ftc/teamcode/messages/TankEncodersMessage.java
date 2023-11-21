package org.firstinspires.ftc.teamcode.messages;

import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;

import java.util.List;

public final class TankEncodersMessage {
    public long timestamp;
    public int[] rawLeftPositions;
    public int[] rawLeftVelocities;
    public int[] rawRightPositions;
    public int[] rawRightVelocities;

    public TankEncodersMessage(List<PositionVelocityPair> left, List<PositionVelocityPair> right) {
        this.timestamp = System.nanoTime();
        this.rawLeftPositions = new int[left.size()];
        this.rawLeftVelocities = new int[left.size()];
        this.rawRightPositions = new int[right.size()];
        this.rawRightVelocities = new int[right.size()];
        for (int i = 0; i < left.size(); i++) {
            this.rawLeftPositions[i] = left.get(i).rawPosition;
            this.rawLeftVelocities[i] = left.get(i).rawVelocity;
        }
        for (int i = 0; i < right.size(); i++) {
            this.rawRightPositions[i] = right.get(i).rawPosition;
            this.rawRightVelocities[i] = right.get(i).rawVelocity;
        }
    }
}
