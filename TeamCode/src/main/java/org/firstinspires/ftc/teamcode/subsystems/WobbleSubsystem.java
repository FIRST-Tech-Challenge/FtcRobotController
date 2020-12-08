package org.firstinspires.ftc.teamcode.subsystems;

import com.technototes.library.hardware.servo.Servo;
import com.technototes.library.subsystem.Subsystem;
import com.technototes.library.subsystem.motor.EncodedMotorSubsystem;
import com.technototes.library.subsystem.servo.ServoSubsystem;

/** Wobble goal manipulator subsystem
 *
 */
public class WobbleSubsystem extends ServoSubsystem {
    public Servo armServo, clawServo;
    public WobbleSubsystem(Servo claw, Servo arm){
        super(claw, arm);
        armServo = arm;
        clawServo = claw;
    }
    //TODO
}
