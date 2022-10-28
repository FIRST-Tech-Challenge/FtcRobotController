package org.firstinspires.ftc.teamcode.opmodes.base;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.components.DriveSystem;
import org.firstinspires.ftc.teamcode.components.PixyCam;
import org.firstinspires.ftc.teamcode.components.Vuforia;


/**
 * Basic OpMode template
 */
@Autonomous(name= "orient", group = "Autonomous")
public abstract class OrientOpMode extends BaseOpMode {

    protected DriveSystem driveSystem;
    protected Vuforia vuforia;
    private boolean stopRequested;
    protected PixyCam pixyCam;
    private PixyCam.Block block;
    private int offset;


    /** Initialization */

    public void init(){
        if(pixyCam == null){
            pixyCam = hardwareMap.get(PixyCam.class, "pixy");
        }
    }

    public void loop(){
        block = pixyCam.GetBiggestBlock();
        offset = pixyCam.offSetX(3);
        String s = block.width + " " + block.height;
        String coords = block.x + ", " + block.y;
        telemetry.addData("block", s);
        telemetry.addData("coords", coords);
        telemetry.addData("offset", offset);
        Log.d("grrrr", offset + "");
        telemetry.update();
        if(offset > 20){
            driveSystem.turn(90, 0.5);
        }

        if(offset < -20){
            driveSystem.turn(-90, 0.5);
        }

        if(offset < 20 || offset > -20){
            driveSystem.setMotorPower(0);
        }
    }
}
