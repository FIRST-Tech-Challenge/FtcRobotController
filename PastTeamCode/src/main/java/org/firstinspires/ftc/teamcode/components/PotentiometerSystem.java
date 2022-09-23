package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.LED;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


public class PotentiometerSystem {

    public static AnalogInput sensorAsAnalogInput0;
    //private ArmSystem armSystem;
    //private static final double TARGET_VOLTAGE = 343;
//    private AnalogInput sensorAsAnalogInput1;
//    private AnalogInput sensorAsAnalogInput2;
//    private AnalogInput sensorAsAnalogInput3;
//    private LED weightIndicatorRed;
//    private LED weightIndicatorGreen;


    public PotentiometerSystem(AnalogInput input0 /*ArmSystem armSystem*/){
        sensorAsAnalogInput0 = input0;
        //this.armSystem = armSystem;
        //init();
    }

    public void init(){
        //double currentVoltage = sensorAsAnalogInput0.getVoltage(); /* + sensorAsAnalogInput1.getVoltage() + sensorAsAnalogInput2.getVoltage() + sensorAsAnalogInput3.getVoltage();*/
        //telemetry.addData("See: ", sensorAsAnalogInput0.getVoltage());
        // through testing - found that voltage reads 1.7 when theres nothing, 0 when there is something, hence the following classifications
        /*while (currentVoltage > TARGET_VOLTAGE){ // Light Block
            armSystem.move_up();
            // the light should be off when there is a light block
        }
        else if(currentVoltage > 5){ // Medium Block
            weightIndicatorRed.enableLight(true);
            weightIndicatorGreen.enableLight(false);
            // red light is on when there is a
        }
        else{ // Light Block
            weightIndicatorRed.enableLight(true);
            weightIndicatorGreen.enableLight(false);
            // light up green
        }
        */
    }
}
