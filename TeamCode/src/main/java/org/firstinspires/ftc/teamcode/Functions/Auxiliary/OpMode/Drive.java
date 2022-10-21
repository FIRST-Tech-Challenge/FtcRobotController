package org.firstinspires.ftc.teamcode.Functions.Auxiliary.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement.Movement;
import org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement.Rotation;
import org.firstinspires.ftc.teamcode.Functions.Auxiliary.DefaultVariables;

@TeleOp(name="Default Name", group="Default Group")
public abstract class Drive extends DefaultVariables {


    /**
     * You should add before the class declaration: @TeleOp(name="Name", group="Group")
     * leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor are already declared, you just have to use hardwareMap.dcMotor.get("name");
     * movement and rotation are also declared and will be initialized if all the motors are not null
     * you will be informed if movement and rotation have been initialised with a message in telemetry during initialization, you can turn it off using telemetrySettings.
     * telemetry.update() is added by default at the end of all methods, it can be turned off using telemetrySettings
     * when importing, if you want to enable/disable generating this comments, just turn on/off Javadoc option
     *
     * This method is called once when the driver first presses the button on the driver station
     * Initialize here any variables you might need
     */
    public abstract void Initialization();

    @Override
    public void init() {
        double startTime = getRuntime();
        Initialization();
        initialiseVariables(startTime);

    }



    /**
     * This method is constantly called after the driver first presses the button on the driver station
     * The robot MUST NOT MOVE during initialization
     * Not often used. You may safely leave this method empty if you don't have a use for it
     * Might be useful to get sensor data
     */
    public abstract void InitializationLoop();
    @Override
    public void init_loop() {
        InitializationLoop();

    }

    /**
     * This method is called once when the driver presses the start button on the driver station
     * You may initialize any variables you might need here as well, but it is recommended you do that in Initialization() instead
     * Might be useful to set the start time for example
     */
    public abstract void GameStart();
    @Override
    public void start() {

        GameStart();

    }

    /**
     * This method is constantly called after the driver presses the start button on the driver station
     * This is the main method of you code
     * The algorithm for moving the robot goes in this method
     * You can now use movement.MoveFull(direction); or rotation.MoveFull(direction) here
     * param "direction" has to be of type MovementFunction.Move
     */
    public abstract void GameLoop();
    @Override
    public void loop() {

        GameLoop();


    }

    /**
     * This method is called once when the driver presses the stop button on the driver station
     * Any unfinished business goes here
     * Might be useful to save a log file for example
     */
    public abstract void GameStop();
    @Override
    public void stop() {

        GameStop();

    }



}
