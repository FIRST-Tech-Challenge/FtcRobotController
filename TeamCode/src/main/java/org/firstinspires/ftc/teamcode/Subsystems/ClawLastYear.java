//package org.firstinspires.ftc.teamcode.Subsystems;
//
//import com.arcrobotics.ftclib.command.SubsystemBase;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.RobotContainer;
//
//
///** Subsystem */
//public class Claw extends SubsystemBase {
//
//    // Local objects and variables here
//    // Right claw
//    private static final double RIGHT_MAX_POS     =  0.4;     // Maximum rotational position of the right servo
//    private static final double RIGHT_MIN_POS     =  0.0;     // Minimum rotational position of the right servo
//    // Left claw
//    private static final double LEFT_MAX_POS     =  1.0;     // Maximum rotational position of the left servo
//    private static final double LEFT_MIN_POS     =  0.6;     // Minimum rotational position of the left servo
//
//    private boolean servoOpen = false;
//    private Servo rightClaw;
//    private Servo leftClaw;
//
//    /** Place code here to initialize subsystem */
//    public Claw() {
//        leftClaw = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "left_claw");
//        rightClaw = RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "right_claw");
//    }
//
//    /** Method called periodically by the scheduler
//     * Place any code here you wish to have run periodically */
//    @Override
//    public void periodic() {
//
//    }
//
//    /**
//     *  Sets both the left and right servos to an open position.
//     */
//    public void ClawOpen(){
//            rightClaw.setPosition(RIGHT_MAX_POS);
//            leftClaw.setPosition(LEFT_MIN_POS);
//            servoOpen = true;
//    }
//
//    /**
//     * Sets both the left and right servos to a closed position.
//     */
//    public void ClawClosed(){
//            rightClaw.setPosition((RIGHT_MIN_POS));
//            leftClaw.setPosition(LEFT_MAX_POS);
//            servoOpen = false;
//    }
//    // place special subsystem methods here
//
//    /**
//     * returns a boolean stating whether the claw has been told to open
//     */
//    public boolean isServoOpen(){
//        return servoOpen;
//    }
//
//}