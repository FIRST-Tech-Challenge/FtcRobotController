package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.BlinkinColour;


/** Subsystem */
public class Blinkin extends SubsystemBase {

    // Create wrist Servo

    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;



    /** Place code here to initialize subsystem */
    public Blinkin() {

        // Creates a Servo using the hardware map
        blinkinLedDriver = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = BlinkinColour.RED_ALLIANCE.getPattern();
       blinkinLedDriver.setPattern(pattern);
    }

    /** Method called periodically by the scheduler
     * Place any code here you wish to have run periodically */
    @Override
    public void periodic() {

    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
        blinkinLedDriver.setPattern(blinkinPattern);
    }

    // Open and close the claw using the enum ClawState
//    public void ControlClaw(ClawState state){
//        clawServo.setPosition(state.getValue());
//    }


}