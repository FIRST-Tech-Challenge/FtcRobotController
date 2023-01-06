package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.hardware.motors.Motor;

/** A component that does not really do much, it is here just as a placeholder
 *
 */
public class ExampleComponent {
    Motor motor;

    public ExampleComponent(Motor motor){
        this.motor = motor;
    }

    public void rotateMotor(double power){
        // Some complicated logic
        this.motor.set(power);
    }
}
