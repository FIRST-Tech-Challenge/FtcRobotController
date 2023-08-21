public class AutonomousMovement {
    public static void main(String[] args) {
        // Simulate moving forward
        System.out.println("Moving forward 12 inches...");
        moveForward(12);

        // Simulate moving back
        System.out.println("Moving back 12 inches...");
        moveBack(12);

        System.out.println("Movement complete.");
    }

    // Simulate moving forward
    public static void moveForward(int distanceInches) {
        System.out.println("Moving forward " + distanceInches + " inches.");
        // Here you would put the actual code to control the robot's movement.
        // For this simulation, we'll just print messages.
    }

    // Simulate moving back
    public static void moveBack(int distanceInches) {
        System.out.println("Moving back " + distanceInches + " inches.");
        // Here you would put the actual code to control the robot's movement back.
        // For this simulation, we'll just print messages.
    }
}
