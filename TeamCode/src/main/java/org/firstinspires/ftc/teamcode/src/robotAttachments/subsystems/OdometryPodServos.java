package org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A class to raise and lower the Odometry servos
 */
public class OdometryPodServos {

    /**
     * The position the right servo must go to to be up
     */
    private static final double rightServoRaisePosition = 0.34;

    /**
     * The position the right servo must go to to be down
     */
    private static final double rightServoLowerPosition = .049;

    /**
     * The position the left servo must go to to be up
     */
    private static final double leftServoRaisePosition = 0.11;

    /**
     * The position the left servo must go to to be down
     */
    private static final double leftServoLowerPosition = 0.37;

    /**
     * The position the horizontal servo must go to to be up
     */
    private static final double horizontalServoRaisePosition = 0.22;
    /**
     * The position the horizontal servo must go to to be down
     */
    private static final double horizontalServoLowerPosition = 0.49;

    /**
     * Horizontal Servo Object
     */
    public final Servo horizontalServo;
    /**
     * Left Servo Object
     */
    public final Servo leftServo;
    /**
     * Right Servo Object
     */
    public final Servo rightServo;

    /**
     * Constructs and initializes servos
     *
     * @param hardwareMap         OpMode hardware map
     * @param rightServoName      right servo name
     * @param leftServoName       left servo name
     * @param horizontalServoName horizontal servo name
     */
    public OdometryPodServos(HardwareMap hardwareMap, String rightServoName, String leftServoName, String horizontalServoName) {
        this.horizontalServo = hardwareMap.servo.get(horizontalServoName);
        this.rightServo = hardwareMap.servo.get(rightServoName);
        this.leftServo = hardwareMap.servo.get(leftServoName);
    }

    /**
     * Lowers the Odometry pods
     */
    public void lower() {
        horizontalServo.setPosition(horizontalServoLowerPosition);
        rightServo.setPosition(rightServoLowerPosition);
        leftServo.setPosition(leftServoLowerPosition);
    }

    /**
     * Raises the odometry pods
     */
    public void raise() {
        horizontalServo.setPosition(horizontalServoRaisePosition);
        rightServo.setPosition(rightServoRaisePosition);
        leftServo.setPosition(leftServoRaisePosition);
    }
}
