package org.firstinspires.ftc.teamcode.TestingOpModes.Examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzLauncherUltimateGoal;
import org.firstinspires.ftc.teamcode.SubSystems.Examples.HzMagazineUltimateGoal;

/**
 * TeleOpMode for Team Hazmat<BR>
 */
@TeleOp(name = "Test_Launcher", group = "Test")
@Disabled
public class Test_LauncherUltimateGoal extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadTestControllerUltimateGoal hzGamepadTestControllerUltimateGoal;
    HzMagazineUltimateGoal hzMagazineUltimateGoal;
    HzLauncherUltimateGoal hzLauncherUltimateGoal;
    double powerLoop = 0.3;
    double velocityLoop = 1200;

    @Override
    public void runOpMode() {
        hzLauncherUltimateGoal = new HzLauncherUltimateGoal(hardwareMap);
        hzMagazineUltimateGoal = new HzMagazineUltimateGoal(hardwareMap);
        hzGamepadTestControllerUltimateGoal = new HzGamepadTestControllerUltimateGoal(gamepad1);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //**** Launcher Actions ****
            hzMagazineUltimateGoal.moveMagazineToLaunch();
            //Launches Ring
            if (hzGamepadTestControllerUltimateGoal.getRightBumperPress()) {
                //TODO : AMJAD : Launch Controller should be used to check if status is good to launch

                //AMJAD : moveMagazinetoLaunch should not be called by right bumper,
                //it is to be done either automatically, or by Y,X,A,B button press.
                //gpMagazine.moveMagazineToLaunch();

                //if (!gpMagazine.isMagazineEmpty()) {
                    hzLauncherUltimateGoal.plungeRingToFlyWheel();
                //}

            }

            //hzLauncher.launcherFlyWheelMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            //hzLauncher.launcherFlyWheelMotor.setPositionPIDFCoefficients(5.0);

            if (hzGamepadTestControllerUltimateGoal.getLeftBumperPress()) {
                maxVelocityTest();
            }

            if (hzGamepadTestControllerUltimateGoal.getButtonXPress()) {
                //hzLauncher.runFlyWheelToTarget(0.7);
                hzLauncherUltimateGoal.runFlyWheelToTarget(1640);
            }

            if (hzGamepadTestControllerUltimateGoal.getButtonBPress()) {
                hzLauncherUltimateGoal.stopFlyWheel();
            }

            /*if (powerLoop >1.0) powerLoop = 1.0;
            if (powerLoop < 0.3) powerLoop = 0.3;
            if (hzGamepadClassic.getButtonAPress()) {
                powerLoop = powerLoop - 0.01;
                hzLauncher.runFlyWheelToTarget(powerLoop);
            }

            if (hzGamepadClassic.getButtonYPress()) {
                powerLoop = powerLoop + 0.01;
                hzLauncher.runFlyWheelToTarget(powerLoop);
            }*/

            if (velocityLoop >2340) velocityLoop = 2340;
            if (velocityLoop < 1200) velocityLoop = 1200;
            if (hzGamepadTestControllerUltimateGoal.getButtonAPress()) {
                velocityLoop = velocityLoop - 10;
                hzLauncherUltimateGoal.runFlyWheelToTarget(velocityLoop);
            }

            if (hzGamepadTestControllerUltimateGoal.getButtonYPress()) {
                velocityLoop = velocityLoop + 10;
                hzLauncherUltimateGoal.runFlyWheelToTarget(velocityLoop);
            }


            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    double currentVelocity, maxVelocity = 0;

    public void maxVelocityTest(){
        //hzLauncher.runFlyWheelToTarget(1.0);
        hzLauncherUltimateGoal.launcherFlyWheelMotor.setPower(1.0);
        currentVelocity = hzLauncherUltimateGoal.launcherFlyWheelMotor.getVelocity();

        if (currentVelocity > currentVelocity) {
            maxVelocity = currentVelocity;
        }

    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("HzDEBUG_FLAG is : ", HzDEBUG_FLAG);
        telemetry.addData("7:24","11/23");
        telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncherUltimateGoal.launcherFlyWheelMotor.isBusy());
        telemetry.addData("Launch Motor Power", hzLauncherUltimateGoal.launcherFlyWheelMotor.getPower());
        telemetry.addData("velocityLoop: ", velocityLoop);
        telemetry.addData("Laucnch Motor Velocity", hzLauncherUltimateGoal.launcherFlyWheelMotor.getVelocity());

        telemetry.addData("launcher MaxVelocity : ",maxVelocity );
        telemetry.addData("launcherRingPlungerServo.getPosition()", hzLauncherUltimateGoal.launcherRingPlungerServo.getPosition());

        switch (hzLauncherUltimateGoal.getLauncherState()){
            case RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case RUNNING_FOR_SUPPLY:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case STOPPED:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_STOPPED");
                break;
            }
        }

    }

}


