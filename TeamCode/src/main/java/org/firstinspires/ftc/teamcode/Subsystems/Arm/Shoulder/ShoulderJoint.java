package org.firstinspires.ftc.teamcode.Subsystems.Arm.Shoulder;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotContainer;


/** Shoulder Subsystem
 * 0° is up*/
public class ShoulderJoint extends SubsystemBase {

    // Create the shoulder motor
    /**0° is up*/
    private final Servo ShoulderServo;

    // used for motion profiling of servo
    TrapezoidProfile profile;
    ElapsedTime timer;


    /** Place code here to initialize subsystem */
    public ShoulderJoint() {

        // Creates a Servo using the hardware map
        ShoulderServo =  RobotContainer.ActiveOpMode.hardwareMap.get(Servo.class, "shoulderServo");

        timer = new ElapsedTime();
        timer.reset();
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

        // if we have a profile to control to, then command servo position
        // based on time elapsed into the profile.  Otherwise do nothing.
        if (profile!=null)
            ShoulderServo.setPosition(profile.calculate(timer.seconds()).position);

    }


    // Turns the Servo a set amount of degrees
    public void RotateTo(int degrees){

        // Converts degrees into 0-1 float
        double servoPos = degrees/270.0;

        // we are about to be commanded a new profile.
        // first determine starting state of new profile.
        // did we previously have a profile? If so, get current state
        // if no profile, simply get current position and assume zero speed.
        TrapezoidProfile.State startState;
        if (profile==null)
            startState = new TrapezoidProfile.State(ShoulderServo.getPosition(), 0.0);
        else
            startState = new TrapezoidProfile.State(ShoulderServo.getPosition(),
                    profile.calculate(timer.seconds()).velocity);

        // make a new profile
        profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(0.20, 0.035),
                new TrapezoidProfile.State(servoPos,0.0),
                startState);

        timer.reset();

        // Set the Servo to ServoPos
        //ShoulderServo.setPosition(servoPos);

    }

    // Sets the Elbow to fixed positions
    public void setPos(ShoulderPosition pos) {RotateTo(pos.getValue());}


}