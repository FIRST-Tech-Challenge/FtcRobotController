package org.firstinspires.ftc.teamcode.opmodes.base;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.Vuforia;
import org.firstinspires.ftc.teamcode.components.Vuforia.CameraChoice;

import java.util.EnumMap;



/**
 * Basic OpMode template
 */

@Disabled
public abstract class TestingOpMode extends BaseOpMode {


    /** Initialization */
    public void init(){
        super.init();
    }

    public void loop(){

        int count = 0;
        if(count == 0 && driveSystem.driveToPositionTicks(300, DriveSystem.Direction.FORWARD, 0.3)){
            count ++;
            telemetry.addData("Status: ", count);
        }
        if(driveSystem.driveToPositionTicks(300, DriveSystem.Direction.BACKWARD, 0.3)){
            count ++;
            telemetry.addData("Status: ", count);
        }
        if(driveSystem.driveToPositionTicks(300, DriveSystem.Direction.LEFT, 0.3)){
            count ++;
            telemetry.addData("Status: ", count);
        }
        if(driveSystem.driveToPositionTicks(300, DriveSystem.Direction.RIGHT, 0.3)){
            count ++;
            telemetry.addData("Status: ", count);
        }
    }

    /** Returns if a stop has been requested or if execution is
     */


    public void stop() {
        super.stop();
    }
}
