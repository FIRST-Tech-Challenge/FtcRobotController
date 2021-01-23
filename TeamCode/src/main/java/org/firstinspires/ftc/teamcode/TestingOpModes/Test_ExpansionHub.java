package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.BatteryChecker;


/**
 * TeleOpMode for Team Hazmat<BR>
 *  Expected behavior.. trigger will pull arm back to park all the time.. To test only key pad, comment out trigger line.
 */
@TeleOp(name = "Test_Battery", group = "Test")
@Disabled
public class Test_ExpansionHub extends LinearOpMode {

    public boolean HzDEBUG_FLAG = true;

    BatteryChecker.BatteryStatus batteryStatus;
    BatteryChecker.BatteryWatcher batteryWatcher;


    public int keyCount = 0;

    @Override
    public void runOpMode() {
        double percent = 0;
        boolean isCharging = false;
        batteryStatus = new BatteryChecker.BatteryStatus(percent,isCharging);
        //batteryStatus.
        //hzArm.initArm(this);
        //Wait for pressing plan on controller
        //waitForStart();
        //keyCount=0;

        //Run Robot based on Gamepad1 inputs
        while (opModeIsActive()) {
            //**** Arm Actions ****
            //Arm Rotation
            //COMMENT THIS LINE OUT TO TEST KEY PAD ONLY.
            //hzArm.moveArmByTrigger(hzGamepadClassic.getLeftTrigger(), this);
            batteryWatcher.updateBatteryStatus(batteryStatus);

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
        telemetry.addData("batteryChecker.percent", batteryStatus.percent);
    }

}


