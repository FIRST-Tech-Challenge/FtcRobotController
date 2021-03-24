package org.firstinspires.ftc.team6220_2020;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.Button;
import org.firstinspires.ftc.team6220_2020.ResourceClasses.PIDFilter;

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
    int[][] startPositions = {/*Position 1: X,Y */{0,0},/*Position 2: X,Y */{4,6}};
    int numStartPositions = startPositions.length - 1;

    //Position values to use in navigation
    double xPos = 0;
    double yPos = 0;
    double lastX = 0;
    double lastY = 0;

    //PID filters for navigation
    PIDFilter translationPID = new PIDFilter(Constants.TRANSLATION_P, Constants.TRANSLATION_I, Constants.TRANSLATION_D);
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

        while(settingUp && !isStopRequested())
        {
            // Select alliance
            if (driver1.isButtonJustPressed(Button.B))
                isRedAlliance = true;
            else if (driver1.isButtonJustPressed(Button.X))
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
            if (driver1.isButtonJustPressed(Button.START))
                settingUp = false;

            // Display the current setup
            telemetry.addData("Is on red alliance: ", isRedAlliance);
            telemetry.addData("Is scoring wobble goal: ", moveWobbleGoal);
            telemetry.addData("Start position: ", matchStartPosition);
            telemetry.update();
            idle();

        }

        //Sets the match start position
        xPos = startPositions[matchStartPosition][0];
        yPos = startPositions[matchStartPosition][1];

    }

    public void driveForwardInches(double targetDistance)
    {
        // Reset motor encoders and return them to RUN_USING_ENCODERS.
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean targetAcquired = false;

        //Starting position to measure distance traveled.
        double startX = xPos;
        double startY = yPos;

        while(!targetAcquired)
        {

            //This calculates the distance traveled in inches
            double distanceTraveled = Math.sqrt(Math.pow((startX - xPos),2) + Math.pow((startY - yPos),2));

            //This adds a value to the PID loop si it can update
            translationPID.roll(distanceTraveled);

            //We drive the mecanum wheels with the PID value
            driveMecanum(0.0,translationPID.getFilteredValue(),0.0);

            // Update positions using last distance measured by encoders (utilizes fact that encoders have been reset to 0).
            xPos = lastX + (double) (Constants.IN_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() +
                    motorBL.getCurrentPosition() - motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / (4));
            yPos = lastY + (double) (Constants.IN_PER_ANDYMARK_TICK * (-motorFL.getCurrentPosition() -
                    motorBL.getCurrentPosition() + motorFR.getCurrentPosition() + motorBR.getCurrentPosition()) / 4);

            telemetry.addData("X Position: ", xPos);
            telemetry.addData("Y Position: ", yPos);
            telemetry.update();
        }
    }

    //Pauses for time milliseconds
    public void pauseMillis(double time){
        double startTime = System.currentTimeMillis();
        while(System.currentTimeMillis() - startTime < time){
            idle();
        }
    }
}
