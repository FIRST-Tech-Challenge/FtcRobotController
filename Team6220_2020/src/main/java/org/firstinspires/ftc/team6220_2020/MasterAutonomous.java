package org.firstinspires.ftc.team6220_2020;

import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;

//todo add is op mode active breakers
public abstract class MasterAutonomous extends MasterOpMode
{
    // Initialize booleans used in runSetup()--------------------------------------------------
    // Determines whether or not we park on the line at the end of autonomous.
    boolean parkOnLine = true;
    // Determines what team we are on.
    boolean isRedAlliance = true;
    // Determines whether or not we move the wobble goal
    boolean moveWobbleGoal = false;
    //-----------------------------------------------------------------------------------------

    // Variables used during setup and running ----------------------------------------------
    // The number of the rings at start up
    int numRings = 0;

    //Start Position Variables. The various start positions are stored in the array start positions are chosen in runSetup.
    int matchStartPosition = 0;
    int[][] startPositions = {/*Position 1: X,Y */{4,6},/*Position 2: X,Y */{4,6}};
    int numStartPositions = startPositions.length - 1;

    //Position values to use in navigation
    double xPos = 0;
    double yPos = 0;
    double lastX = 0;
    double lastY = 0;

    //PID filters for navigation
    //PIDFilter translationPID;
    //-----------------------------------------------------------------------------------------

    // Allows the 1st driver to decide which autonomous routine should be run using gamepad input
    void runSetup()
    {
        // Creates the telemetry log
        telemetry.log().add("Red / Blue = B / X");
        telemetry.log().add("Increase / Decrease Delay = DPad Up / Down");
        telemetry.log().add("Score / Not Score Wobble Goal = Toggle Y");
        telemetry.log().add("Toggle Start Position = Toggle X");
        telemetry.log().add("Press Start to exit setup.");

        boolean settingUp = true;

        while(settingUp && opModeIsActive())
        {
            driver1.update();
            driver2.update();
            // Select alliance
            if (driver1.isButtonPressed(Button.B))
                isRedAlliance = true;
            else if (driver1.isButtonPressed(Button.X))
                isRedAlliance = false;

            //Toggles through moving wobble goal
            if(driver1.isButtonJustReleased(Button.Y))
                moveWobbleGoal = !moveWobbleGoal;

            //Toggles through start positions
            if(driver1.isButtonJustPressed(Button.X)){
                matchStartPosition++;
                if(matchStartPosition > numStartPositions){
                    matchStartPosition = 0;
                }
            }

            // If the driver presses start, we exit setup.
            if (driver1.isButtonJustPressed(Button.A) || opModeIsActive())
                settingUp = false;

            // Display the current setup
            telemetry.addData("a just pressed", driver1.getLeftStickX());
            telemetry.addData("a presses", driver1.isButtonPressed(Button.A));
            telemetry.addData("Is on red alliance: ", isRedAlliance);
            telemetry.addData("Is scoring wobble goal: ", moveWobbleGoal);
            telemetry.addData("Start position: ", matchStartPosition);
            telemetry.update();
            idle();

        }

        telemetry.clearAll();
        telemetry.log().clear();
        telemetry.addData("State: ", "waitForStart()");
        telemetry.update();
        //Sets the match start position
        xPos = startPositions[matchStartPosition][0];
        yPos = startPositions[matchStartPosition][1];

    }
}