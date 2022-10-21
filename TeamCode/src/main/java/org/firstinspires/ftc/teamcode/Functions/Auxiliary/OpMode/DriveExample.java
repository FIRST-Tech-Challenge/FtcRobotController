package org.firstinspires.ftc.teamcode.Functions.Auxiliary.OpMode;

import org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement.MovementFunction;

public class DriveExample extends Drive {


    /**
     * You should add before the class declaration: @TeleOp(name="Name", group="Group")
     * leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor are already declared, you just have to use hardwareMap.dcMotor.get("name");
     * movement and rotation are also declared and will be initialized if all the motors are not null
     * you will be informed if movement and rotation have been initialised with a message in telemetry during initialization, you can turn it off using telemetrySettings.
     * telemetry.update() is added by default at the end of all methods, it can be turned off using telemetrySettings
     * <p>
     * <p>
     * This method is called once when the driver first presses the button on the driver station
     * Initialize here any variables you might need
     */
    @Override
    public void Initialization() {
        leftFrontMotor = hardwareMap.dcMotor.get("INSERT NAME HERE");
        leftBackMotor = hardwareMap.dcMotor.get("INSERT NAME HERE");
        rightBackMotor = hardwareMap.dcMotor.get("INSERT NAME HERE");
        rightFrontMotor = hardwareMap.dcMotor.get("INSERT NAME HERE");
    }

    /**
     * This method is constantly called after the driver first presses the button on the driver station
     * The robot MUST NOT MOVE during initialization
     * Not often used. You may safely leave this method empty if you don't have a use for it
     * Might be useful to get sensor data
     */
    @Override
    public void InitializationLoop() {
        // you don't need to use all methods, you can leave this empty
    }

    /**
     * This method is called once when the driver presses the start button on the driver station
     * You may initialize any variables you might need here as well, but it is recommended you do that in Initialization() instead
     * Might be useful to set the start time for example
     */
    @Override
    public void GameStart() {
        // you don't need to use all methods, you can leave this empty
    }

    /**
     * This method is constantly called after the driver presses the start button on the driver station
     * This is the main method of you code
     * The algorithm for moving the robot goes in this method
     * You can now use movement.MoveFull(direction); or rotation.MoveFull(direction) here
     * param "direction" has to be of type MovementFunction.Move
     */
    @Override
    public void GameLoop() {
        if(gamepad1.dpad_up){
            movement.MoveFull(MovementFunction.Move.UP);
        }
        else if(gamepad1.dpad_down){
            movement.MoveFull(MovementFunction.Move.DOWN);
        }
        else if(gamepad1.dpad_left){
            rotation.MoveFull(MovementFunction.Move.LEFT);
        }
        else if(gamepad1.dpad_right){
            rotation.MoveFull(MovementFunction.Move.RIGHT);
        }
    }

    /**
     * This method is called once when the driver presses the stop button on the driver station
     * Any unfinished business goes here
     * Might be useful to save a log file for example
     */
    @Override
    public void GameStop() {
        movement.Stop();
    }
}
