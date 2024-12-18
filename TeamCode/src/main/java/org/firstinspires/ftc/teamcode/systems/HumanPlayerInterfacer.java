package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PWMOutput;

import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.Settings;

public class HumanPlayerInterfacer {
    private PWMOutput led;

    // Constructor: Initializes the LED using the HardwareMap
    public HumanPlayerInterfacer(BaseRobot baseRobot) {
        led = baseRobot.hardwareMap.get(PWMOutput.class, Settings.Hardware.IDs.LED);
        clear(); // Ensure the LED starts off
    }

    // Method to turn off the LED
    public void clear() {
        setLEDColor(0); // Turn off the LED
    }

    // Method to change the LED to red (chamber mode)
    public void chamber() {
        setLEDColor(500); // Set the LED to red
    }

    // Method to change the LED to green (basket mode)
    public void basket() {
        setLEDColor(1500); // Set the LED to green
    }

    // Helper method to set the LED color using PWM
    private void setLEDColor(int pulseWidth) {
        // Configure the PWM signal
        led.setPulseWidthPeriod(20000); // 20ms frame period (standard for servos/LEDs)
        led.setPulseWidthOutputTime(pulseWidth); // Set the pulse width for desired output
    }
}
