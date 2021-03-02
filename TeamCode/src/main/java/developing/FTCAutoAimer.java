package developing;


public class FTCAutoAimer {
    // Set these to their actual values
    public final double boardAngle = 20 * Math.PI/180; // degrees -> radians
    public final double goalFromLeft = 0.9; // meters
    public final double goalHeight = 0.9; // meters
    public final double shooterHeight = 0.25; // meters
    public final double shooterWheelRadius = 0.05; // meters

//    public static void main(String[] args) {
//        setSpeed(calcSpeed(1.5, 0.9));
//    }

    public double calcSpeed(double disFromFront, double disFromLeft) {
        double disToGoal = Math.sqrt(Math.pow(disFromFront, 2) + Math.pow(disFromLeft - goalFromLeft, 2));
        double deltaHeight = goalHeight - shooterHeight;
        double linearSpeed = disToGoal/Math.cos(boardAngle) * Math.sqrt(4.9/(disToGoal * Math.tan(boardAngle) - deltaHeight));
        return linearSpeed/shooterWheelRadius;
    }

//    public void setSpeed(double speed) {
//        // The PID Controller
//        System.out.println("Set speed to " + speed + " rad/s, or " + (speed*shooterWheelRadius) + " m/s");
//    }
}