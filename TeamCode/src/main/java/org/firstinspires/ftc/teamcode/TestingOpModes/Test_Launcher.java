package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;

/**
 * TeleOpMode for Team Hazmat<BR>
 */
@TeleOp(name = "Test_Laucher", group = "Test")
public class Test_Launcher extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassic hzGamepadClassic;
    Launcher hzLauncher;
    double powerLoop = 0.3;

    @Override
    public void runOpMode() {
        hzLauncher = new Launcher(hardwareMap);
        hzGamepadClassic = new HzGamepadClassic(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        hzLauncher.initLauncher();

        //Wait for pressing plan on controller
        waitForStart();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //**** Launcher Actions ****
            //Launches Ring
            if (hzGamepadClassic.getRightBumperPress()) {
                //TODO : AMJAD : Launch Controller should be used to check if status is good to launch

                //AMJAD : moveMagazinetoLaunch should not be called by right bumper,
                //it is to be done either automatically, or by Y,X,A,B button press.
                //gpMagazine.moveMagazineToLaunch();

                //if (!gpMagazine.isMagazineEmpty()) {
                    hzLauncher.plungeRingToFlyWheel();
                //}

            }

            //hzLauncher.launcherFlyWheelMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
            //hzLauncher.launcherFlyWheelMotor.setPositionPIDFCoefficients(5.0);


            if (hzGamepadClassic.getLeftBumperPress()) {
                maxVelocityTest();
            }

            if (hzGamepadClassic.getButtonXPress()) {
                hzLauncher.runFlyWheelToTarget(0.7);
            }

            if (hzGamepadClassic.getButtonBPress()) {
                hzLauncher.stopFlyWheel();
            }

            if (powerLoop >1.0) powerLoop = 1.0;
            if (powerLoop < 0.3) powerLoop = 0.3;
            if (hzGamepadClassic.getButtonAPress()) {
                powerLoop = powerLoop - 0.01;
                hzLauncher.runFlyWheelToSupply(powerLoop);
            }

            if (hzGamepadClassic.getButtonYPress()) {
                powerLoop = powerLoop + 0.01;
                hzLauncher.runFlyWheelToSupply(powerLoop);
            }

            if(HzDEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }
        }
    }

    double currentVelocity, maxVelocity = 0;

    public void maxVelocityTest(){
        hzLauncher.runFlyWheelToTarget(1.0);
        currentVelocity = hzLauncher.launchMotorVelocity;

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
        telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("Launch Motor Power", hzLauncher.launcherFlyWheelMotor.getPower());
        telemetry.addData("Laucnch Motor Velocity", hzLauncher.launcherFlyWheelMotor.getVelocity());

        telemetry.addData("launcher MaxVelocity : ",maxVelocity );
        telemetry.addData("launcherRingPlungerServo.getPosition()", hzLauncher.launcherRingPlungerServo.getPosition());

        switch (hzLauncher.getLauncherState()){
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


