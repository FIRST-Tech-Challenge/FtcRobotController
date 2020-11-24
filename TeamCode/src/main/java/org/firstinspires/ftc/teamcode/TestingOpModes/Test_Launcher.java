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

            if (hzGamepadClassic.getButtonAPress()) {
                hzLauncher.runFlyWheelToTarget(0.7);
            }

            if (hzGamepadClassic.getButtonBPress()) {
                hzLauncher.stopFlyWheel();
            }

            if (hzGamepadClassic.getButtonXPress()) {
                hzLauncher.runFlyWheelToSupply(0.2);
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

        telemetry.addData("launcherFlyWheelMotor.isBusy()", hzLauncher.launcherFlyWheelMotor.isBusy());
        telemetry.addData("launcherRingPlungerServo.getPosition()", hzLauncher.launcherRingPlungerServo.getPosition());

        switch (hzLauncher.getLauncherState()){
            case FLYWHEEL_RUNNING_FOR_TARGET:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case FLYWHEEL_RUNNING_FOR_SUPPLY:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_RUNNING_FOR_SUPPLY");
                break;
            }
            case FLYWHEEL_STOPPED:  {
                telemetry.addData("hzLauncher.getLauncherState()", "FLYWHEEL_STOPPED");
                break;
            }
        }

    }

}


