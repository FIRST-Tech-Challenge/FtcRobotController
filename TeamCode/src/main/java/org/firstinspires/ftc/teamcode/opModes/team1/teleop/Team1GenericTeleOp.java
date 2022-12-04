package org.firstinspires.ftc.teamcode.opModes.team1.teleop;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.components.ExampleComponent;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TeamColour;

public class Team1GenericTeleOp {
    TeamColour teamColour;
    ExampleComponent exampleComponent;

    public void setup(TeamColour teamColor){
        this.teamColour = teamColor;
        // Get the third motor as a spinner motor
        this.exampleComponent = new ExampleComponent(HardwareMapContainer.motor3);
    }

    public void every_tick(){
        // As an example,
        // We want to spin the motor in one direction if we are on the blue team and in the other if we are on the red team
        if (teamColour == TeamColour.BLUE) exampleComponent.rotateMotor(1);
        else exampleComponent.rotateMotor(-1);
    }
}
