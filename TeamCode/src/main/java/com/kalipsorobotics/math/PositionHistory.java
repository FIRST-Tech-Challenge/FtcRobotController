package com.kalipsorobotics.math;

public class PositionHistory {

    private Position currentPosition;
    private Velocity currentVelocity;

    public PositionHistory() {

    }

    public PositionHistory(Position currentPosition, Velocity currentVelocity) {
        this.currentPosition = currentPosition;
        this.currentVelocity = currentVelocity;
    }

    public PositionHistory(PositionHistory positionHistory) {
        this.currentPosition = positionHistory.currentPosition;
        this.currentVelocity = positionHistory.currentVelocity;
    }

    public Position getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(Position currentPosition) {
        this.currentPosition = currentPosition;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    public void setCurrentVelocity(Velocity currentVelocity) {
        this.currentVelocity = currentVelocity;
    }

    @Override
    public String toString() {
        return "PositionHistory{" +
                "currentPosition=" + currentPosition +
                ", currentVelocity=" + currentVelocity +
                '}';
    }
}
