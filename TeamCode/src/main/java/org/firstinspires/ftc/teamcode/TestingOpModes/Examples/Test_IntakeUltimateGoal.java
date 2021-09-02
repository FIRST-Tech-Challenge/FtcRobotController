package org.firstinspires.ftc.teamcode.TestingOpModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzIntakeUltimateGoal;

/**
 * TeleOpMode for Team Hazmat<BR>
 */
@TeleOp(name = "Test_Intake", group = "Test")
@Disabled
public class Test_IntakeUltimateGoal extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadTestControllerUltimateGoal hzGamepadTestControllerUltimateGoal;
    HzIntakeUltimateGoal hzIntakeUltimateGoal;

    @Override
    public void runOpMode() {
        hzIntakeUltimateGoal = new HzIntakeUltimateGoal(hardwareMap);
        hzGamepadTestControllerUltimateGoal = new HzGamepadTestControllerUltimateGoal(gamepad1);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        hzIntakeUltimateGoal.initIntake();

        //Wait for pressing plan on controller
        waitForStart();
        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //Run per Gamepad input
            //hzGamepad1.runSubsystemByGamepadInput(hzChassis, hzArm, hzIntake, hzLauncher, hzMagazine, hzLauncherController);

            //****Intake Actions****
            //Run Intake motors - start when Dpad_down is pressed once, and stop when it is pressed again
            //TODO : AMJAD : Check if this logic works to keep motor running till Dpad_down is toggled.
            //May need to save state of last time button pressed and runmotor based on button last pressed state
            if (hzGamepadTestControllerUltimateGoal.getDpad_downPress()) {
                if(hzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.STOPPED) {
                    //if(gpMagazine.moveMagazineToCollect()) {
                        hzIntakeUltimateGoal.runIntakeMotor();
                    //}
                } else if(hzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.RUNNING) {
                    hzIntakeUltimateGoal.stopIntakeMotor();
                }
            }

            //Reverse Intake motors and run - in case of stuck state)
            //TODO : AMJAD : This probably works, since we want to run the Intake motor in reverse
            // only as long as te Dpad_up remains pressed (different from Dpad_down behavior, which is a toggle)
            if (hzGamepadTestControllerUltimateGoal.getDpad_upPersistent()) {
                hzIntakeUltimateGoal.reverseIntakeMotor();
            } else if (hzIntakeUltimateGoal.getIntakeState() == HzIntakeUltimateGoal.INTAKE_MOTOR_STATE.REVERSING){
                hzIntakeUltimateGoal.stopIntakeMotor();
            }

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("7:18","11/23");
        telemetry.addData("hzGamepad1.getDpad_downPress()", hzGamepadTestControllerUltimateGoal.getDpad_downPress());
        telemetry.addData("hzGamepad1.getDpad_upPress()", hzGamepadTestControllerUltimateGoal.getDpad_upPress());
        telemetry.addData("intakeMotor.isBusy()", hzIntakeUltimateGoal.intakeMotor.isBusy());
        switch (hzIntakeUltimateGoal.getIntakeState()){
            case RUNNING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_RUNNING");
                break;
            }
            case STOPPED: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_STOPPED");
                break;
            }
            case REVERSING: {
                telemetry.addData("hzIntake.getIntakeState()", "INTAKE_MOTOR_REVERSING");
                break;
            }
        }
    }

}


