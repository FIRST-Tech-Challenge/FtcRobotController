package teamcode.common;

import teamcode.common.PurePursuit.MathFunctions;

public class RobotPositionStateUpdater {
    RobotPositionState state;
    public static class RobotPositionState {
        private Vector2D position;
        private double rotation;
        private long positionUpdateTime;
        private Vector2D velocity;
        private double angularVelocity;
        public RobotPositionState(Vector2D position, Vector2D velocity, double rotation, double angularVelocity) {
            this.position = position;
            this.velocity = velocity;
            this.rotation = rotation;
            this.angularVelocity = angularVelocity;
        }
        public Vector2D getPosition() {
            return position;
        }
        public double getRotation() {
            return rotation;
        }
        public double getAngularVelocity() {
            return angularVelocity;
        }
        public Vector2D getVelocity() {
            return velocity;
        }
        public RobotPositionState copy() {
            return new RobotPositionState(this.position, this.velocity, this.rotation, this.angularVelocity);
        }

        public String toString(){
            return "Position: "  + position.toString() + "\n" +
                    "Velocity: " + velocity.toString() + "\n" +
                    "rotation: " + rotation + "\n" +
                    "angular Velocity: " + angularVelocity + "\n";
        }

        public String loggingToString(){
            return position.getX() + "," + position.getY() +"," + velocity.getX() + velocity.getY() + "," + rotation + "," + angularVelocity;
        }

    }

    public RobotPositionStateUpdater() {
        this(new Vector2D(0,0), new Vector2D(0,0), 0, 0);
    }
    public RobotPositionStateUpdater(Vector2D position, Vector2D velocity, double rotation, double angularVelocity) {
        state = new RobotPositionState(position, velocity, rotation, angularVelocity);
    }
    public synchronized RobotPositionState getCurrentState() {
        return state.copy();
    }
    public void resetUpdateTime() {
        state.positionUpdateTime = System.nanoTime();
    }
    private double updatePositionTime() {
        long newTime = System.nanoTime();
        double elapsedNanos = newTime - state.positionUpdateTime;
        state.positionUpdateTime = newTime;
        return elapsedNanos/1000000.0;
    }
    public synchronized void updateDelta(double deltaX, double deltaY, double deltaPhi,
                                         double deltaVx, double deltaVy, double omega) {
        updatePositionTime();
        state.position = state.position.add(new Vector2D(deltaX, deltaY));
        state.rotation = MathFunctions.angleWrap(state.rotation + deltaPhi);
        state.velocity = new Vector2D(deltaVx, deltaVy);
        state.angularVelocity = omega;
    }

    public synchronized void updateState(double deltaX, double deltaY, double deltaPhi,
                                         double deltaVx, double deltaVy, double omega) {
        updatePositionTime();
        state.position = new Vector2D(deltaX, deltaY);
        state.rotation =  MathFunctions.angleWrap(deltaPhi);
        state.velocity = new Vector2D(deltaVx, deltaVy);
        state.angularVelocity = omega;
    }
}
