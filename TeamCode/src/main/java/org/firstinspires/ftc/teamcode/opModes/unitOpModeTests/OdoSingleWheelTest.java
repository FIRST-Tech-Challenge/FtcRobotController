package org.firstinspires.ftc.teamcode.opModes.unitOpModeTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.odometry.FreeSpinOdoWheel;
import org.firstinspires.ftc.teamcode.utility.pose;

import java.util.ArrayList;


@TeleOp(name="OdoSingleTest", group="Iterative Opmode")

public class OdoSingleWheelTest extends OpMode
{
    DcMotor wheel;
    FreeSpinOdoWheel odowheel;

    int cyclesSinceLastUpdate = 0;
    ArrayList<Integer> recordCyclesToUpdate = new ArrayList();

    @Override
    public void init() {
        wheel = hardwareMap.get(DcMotor.class, "horizontal");
        odowheel = new FreeSpinOdoWheel(new pose(0,0,0), wheel);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        MotorConfigurationType type = wheel.getMotorType();

        double prevDist = wheel.getCurrentPosition();

        odowheel.updateDelta();
        odowheel.getDeltaPosition();

        // Show the elapsed game time and wheel power.
        telemetry.addData("Motor type", type);
        telemetry.addData("Status", "Wheel Position: " + String.format("%d", wheel.getCurrentPosition()));
        telemetry.addData("Status", "Wheel Distance: " + String.format("%.1f", odowheel.totalDistTravelled));

        /*//did it update?
        if(wheel.getCurrentPosition() == prevDist){//no
            cyclesSinceLastUpdate ++;
        }else{//yes
            recordCyclesToUpdate.add(cyclesSinceLastUpdate);
            if(recordCyclesToUpdate.size()>1000){
                recordCyclesToUpdate.remove(0);
            }
            cyclesSinceLastUpdate=0;
        }

        //average
        double sum = 0;
        for(int c:recordCyclesToUpdate){
            sum+=c;
        }
        telemetry.addData("cycles taken to update", sum/recordCyclesToUpdate.size());*/
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "done! Great job!");
    }

}



