package org.firstinspires.ftc.teamcode.Functions.Auxiliary.Autonomous;

public class AutoExample extends Auto{

    /**
     * You should add before the class declaration: @Autonomous(name="Name", group="Group")
     * leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor are already declared, you just have to use hardwareMap.dcMotor.get("name");
     * movement and rotation are also declared and will be initialized if all the motors are not null
     * you will be informed if movement and rotation have been initialised with a message in telemetry during initialization, you can turn it off using telemetrySettings.
     * telemetry.update() is added by default at the end of all methods, it can be turned off using telemetrySettings
     * when importing, if you want to enable/disable generating this comments, just turn on/off Javadoc option
     * <p>
     * This method is called once when the driver first presses the button on the driver station
     * Initialize here any variables you might need
     */
    @Override
    public void Initialization() {

    }

    /**
     * This method is constantly called after the driver first presses the button on the driver station
     * The robot MUST NOT MOVE during initialization
     * Not often used. You may safely leave this method empty if you don't have a use for it
     * Might be useful to get sensor data
     */
    @Override
    public void InitializationLoop() {

    }

    /**
     * This method is called once when the driver presses the start button on the driver station
     * <p>
     * Because you are writing an autonomous class, this should be the main method of you code
     * The algorithm for moving the robot goes in this method
     * You can now use movement.MoveFull(direction); or rotation.MoveFull(direction) here
     * param "direction" has to be of type MovementFunction.Move
     * It is recommended you use this method first when programming the robot.
     * You can use sleep() here
     */
    @Override
    public void GameStart() {

    }

    /**
     * This method is constantly called after the driver presses the start button on the driver station and after GameStart()
     * Use this method if the robot finished what it was doing in GameStart() and you want it to do more
     */
    @Override
    public void GameLoop() {

    }
}
