package org.firstinspires.ftc.teamcode.Libs.AR;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/*--------------------------------------------------------------------------------------------------
 * This is an example OpMode created to show how the AR_Arm Class works, designed for the 2024-2025
 * Aerospace Robotics Robot Arm.
 *
 * Creation Date: 11/3/2024
 ---------------------------------------------------------------------------------------------------
*/
@TeleOp(name="ArmTestRedo with Lights: Linear OpMode", group="Linear OpMode")
public class ArmTestRedo_OpMode_Linear extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

//    DigitalChannel digitalTouch;  // Digital channel Object

    public static final String TAG = "AR_Experimental";

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        AR_Arm arm;
//        AR_Light light;

        // get a reference to our touchSensor object.
//        digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Wait for the game to start (driver presses START)
        waitForStart();
        if (isStopRequested()) return;
        runtime.reset();

        // Instantiate Arm & Light class
        arm = new AR_Arm(this);
//        light = new AR_Light("status_light", this );

        // Place Arm in starting rest position
        //        arm.setArmStartPos( );

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ===== CHECK FOR INPUTS FROM GAMEPADS, ETC. ==========================================
            // We should perform all our user input checks here. Every loop, we should determine if the
            // user has input anything.
            if (gamepad1.triangle || gamepad1.y) {
                telemetry.addData("Status","GP1:Triangle (Light: Police)");
//                light.policeLights();
            }
            if (gamepad1.square || gamepad1.a) {
                telemetry.addData("Status","GP1:Square (setArmDeployPos) Light: Orange");

                // Set Arm into Deploy position.
                arm.setArmDeployPos();
//                light.customLight(AR_Light.GB_CLR_ORANGE);
            }
            if (gamepad1.circle || gamepad1.b) {
                telemetry.addData("Status","GP1:Circle (setArmRestPos)");

                // Set Arm into Rest position.
                arm.setArmActivePos( );
//                light.customLight(AR_Light.GB_CLR_SAGE);
            }
            if (gamepad1.cross || gamepad1.a) {
                telemetry.addData("Status","GP1:Cross (setArmGrabPos)");

                // Set Arm into GRAB position.
                arm.setArmGrabPos( );
//                light.customLight(AR_Light.GB_CLR_AZURE);
            }
            if (gamepad1.dpad_down) {
                telemetry.addData("Status","GP1:DPD (Light: Strobe)");
//                light.strobeLights(AR_Light.GB_CLR_RED, AR_Light.GB_CLR_OFF, 1000, 100);
            }
            if (gamepad1.dpad_up) {
                telemetry.addData("Status","GP1:DPU (Light: Rainbow)");
//                light.rainbowLights();
            }
            if (gamepad1.dpad_left) {
                telemetry.addData("Status","GP1:DPL (Light: Wave)");
//                light.waveLights();
            }
            if (gamepad1.dpad_right) {
                telemetry.addData("Status","GP1:DPR (Light: Red (Custom))");
//                light.customLight(AR_Light.GB_CLR_RED);
            }

//            if (!digitalTouch.getState()) {
//                telemetry.addData("Button", "NOT PRESSED");
//            } else {
//                telemetry.addData("Button", "PRESSED");
//            }

            // ===== RUN ROBOT MECHANICAL UPDATES ==================================================
            // This section is for code that needs to run every loop, even if there is not any user
            // input. For example, since some of our Arm movement controls are "Push a button and
            // forget", it is important that the Arm can update (PID Controller, etc.) even when
            // someone is not pressing a control.
            arm.updateArmPos();
//            light.updateLight();

            // ===== TELEMETRY =====================================================================
            // Show the elapsed game time and other data needed.
            telemetry.addData("Status","Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}