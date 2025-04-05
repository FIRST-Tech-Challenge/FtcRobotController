package com.kalipsorobotics.math;

public class PositionHistory {

    private Position currentPosition;
    private Velocity currentVelocity;
    private Velocity relativeDelta;

    public PositionHistory() {}

    public Position getCurrentPosition() {
        return currentPosition;
    }

    public void setCurrentPosition(Position currentPosition) {
        this.currentPosition = currentPosition;
    }

    public Velocity getCurrentVelocity() {
        return currentVelocity;
    }

    public void setCurrentVelocity(Velocity relativeDelta, double timeElapsed) {
        this.relativeDelta = relativeDelta;
        Velocity currentVelocity = this.currentVelocity.divide(timeElapsed);
        this.currentVelocity = currentVelocity;
    }

    public String toStringCSV() {
        return currentPosition.getX() + ", " +
                currentPosition.getY() + ", " +
                currentPosition.getTheta() + ", " +
                relativeDelta.getTheta();

    }

    @Override
    public String toString() {
        return "PositionHistory{" +
                "currentPosition=" + currentPosition +
                ", currentVelocity=" + currentVelocity +
                '}';
    }
}
