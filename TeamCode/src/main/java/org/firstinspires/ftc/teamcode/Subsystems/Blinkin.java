package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.utility.BlinkinColour;

/**
 * The Blinkin class is a subsystem that controls the Rev Blinkin LED driver.
 * It extends the SubsystemBase class from FTCLib.
 */
public class Blinkin extends SubsystemBase {

    // Instance of the RevBlinkinLedDriver
    private RevBlinkinLedDriver blinkinLedDriver;
    // Current pattern of the Blinkin LED driver
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    /**
     * Constructor for the Blinkin class.
     * Initializes the Blinkin LED driver and sets the initial pattern.
     */
    public Blinkin() {
        // Creates a Blinkin LED driver using the hardware map
        blinkinLedDriver = RobotContainer.ActiveOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        // Sets the initial pattern to RED_ALLIANCE
        pattern = BlinkinColour.RED_ALLIANCE.getPattern();
        blinkinLedDriver.setPattern(pattern);
    }

    /**
     * The periodic method is called periodically by the scheduler.
     * This method is empty as the Blinkin subsystem does not require periodic actions.
     */
    @Override
    public void periodic() {

        // set blinkin for team color
        if (RobotContainer.isRedAlliance)
            setPattern(BlinkinColour.RED_ALLIANCE.getPattern());
        else
            setPattern(BlinkinColour.BLUE_ALLIANCE.getPattern());

    }

    /**
     * Sets the pattern of the Blinkin LED driver.
     *
     * @param blinkinPattern The pattern to set on the Blinkin LED driver.
     */
    public void setPattern(RevBlinkinLedDriver.BlinkinPattern blinkinPattern) {
        blinkinLedDriver.setPattern(blinkinPattern);
    }
}
