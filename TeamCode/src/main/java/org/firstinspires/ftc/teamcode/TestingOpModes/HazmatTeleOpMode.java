package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubSystems.HzChassisClassic;
import org.firstinspires.ftc.teamcode.SubSystems.HzGamepadClassic;

/**
 * TeleOpMode for Team Hazmat<BR>
 * Includes autoplacement routince for automatically placing block on tower
 */
@TeleOp(name = "TestOpMode:HazmatTeleOpMode", group = "TestOpMode")
@Disabled
public class HazmatTeleOpMode extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    HzGamepadClassic hzGamepad;
    HzChassisClassic hzChassisClassic;



    @Override
    public void runOpMode() {

        //Instantiate Subsystems : Chassis, Arm, Intake, Gamepad1
        hzChassisClassic = new HzChassisClassic(hardwareMap);

        hzGamepad = new HzGamepadClassic(gamepad1,this);

        telemetry.addData("Hazmat TeleOp Mode", "v:1.0");

        //Wait for pressing plan on controller
        waitForStart();

        //Initialize on press of play
        hzChassisClassic.initChassis();

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //Run per Gamepad input
            hzGamepad.runByGamepadInputClassicChassis(hzChassisClassic);

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
        telemetry.addData("backRightDrive.getCurrentPosition()", hzChassisClassic.backRight.getCurrentPosition());
        telemetry.addData("backLeftDrive.getCurrentPosition()", hzChassisClassic.backLeft.getCurrentPosition());
        telemetry.addData("frontRightDrive.getCurrentPosition()", hzChassisClassic.frontRight.getCurrentPosition());
        telemetry.addData("frontLeftDrive.getCurrentPosition()", hzChassisClassic.frontLeft.getCurrentPosition());
        telemetry.addData("hzGamepad1.getLeftTrigger()", hzGamepad.getLeftTrigger());


    }

}


