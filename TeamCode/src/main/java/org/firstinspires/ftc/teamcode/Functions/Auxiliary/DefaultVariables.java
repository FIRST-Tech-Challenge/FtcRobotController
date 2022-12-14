package org.firstinspires.ftc.teamcode.Functions.Auxiliary;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement.Movement;
import org.firstinspires.ftc.teamcode.Functions.Auxiliary.Movement.Rotation;
import org.firstinspires.ftc.teamcode.Functions.Auxiliary.OpMode.Drive;

public abstract class DefaultVariables extends OpMode {
    /**
     * Default variables for dc motor used in movement.
     */
    public DcMotor leftFrontMotor = null,
            rightFrontMotor = null,
            leftBackMotor = null,
            rightBackMotor = null;
    /**
     * Default variable for moving the robot.
     */
    public Movement movement = null;

    /**
     * Default variable for roting the robot.
     */
    public Rotation rotation = null;



    public TelemetrySettings telemetrySettings;
    /**
     * You can turn off the initial test check feedback and telemetry.update() on individual methods of your choosing
     */
    class TelemetrySettings{
        public boolean showTestFeedback = true,
                showInitialization= true,
                showInitializationLoop = true,
                showGameLoop = true,
                showGameStart = true,
                showGameEnd = true;
    }


    public String testFeedback="";

    public final void initialiseVariables(double startTime){
        if(isNull(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor)){
            testFeedback+="Check 1: Failed X. One or all of the default variables for dcmotor used in driving (leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor) are null. It will not initialize default movement clases (movement, rotate).\n";
        }
        else{
            testFeedback+="Check 1: Succesfull O. All of the default variables for dcmotor used in driving (leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor) are declared. \n";
            movement = new Movement(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            rotation = new Rotation(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
            if(isNull(movement, rotation)){
                testFeedback+="Check 2: Failed X. One or all of the default variables for movement (movement, rotation) are null due to an internal error.\n";
            }
            else{
                testFeedback+="Check 2: Succesfull O. All of the default variables for movement (movement, rotation) have been successfully initialised.\n";
            }
        }
        double endTime = getRuntime();
        testFeedback+="Initialization took "+(endTime-startTime)+" seconds.\n";


        if(telemetrySettings.showTestFeedback) {
            telemetry.addLine(testFeedback);
        }
    }

    /**
     * I need this to check if some variables are null, you may use it as well
     * @param objects
     * @return
     */
    boolean isNull(Object... objects){
        boolean bool = false;
        for(Object object : objects){
            bool = bool || (object==null);
        }
        return bool;
    }
}
