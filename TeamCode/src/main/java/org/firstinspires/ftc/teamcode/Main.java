package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.HashMap;

@TeleOp
public class Main extends LinearOpMode {
    public static class Data_Update extends Thread {

        public volatile boolean active = true;
        Main main = new Main();
        public void run() {
            if(active){
                while(active){
                    /*
                    Debug Values, shows on driver hub during run
                     */
                    main.addData("Controller 1 left stick", (Main.GetControllerDataItem("LeftStickX1") + "," + Main.GetControllerDataItem("LeftStickY1")));
                    main.addData("Controller 1 right stick", (Main.GetControllerDataItem("RightStickX1") + "," + Main.GetControllerDataItem("RightStickY1")));
                    main.addData("Controller 1 left trigger", (float)Main.GetControllerDataItem("LeftTrigger1"));
                    main.addData("Controller 1 right trigger", (float)Main.GetControllerDataItem("RightTrigger1"));
                    main.addData("Controller 1 left bumper", (Main.GetControllerDataItem("LeftBumper1") == 1f));
                    main.addData("Controller 1 right bumper", (Main.GetControllerDataItem("RightBumper1") == 1f));
                    main.addData("Controller 1 x", (Main.GetControllerDataItem("X1") == 1f));
                    main.addData("Controller 1 y", (Main.GetControllerDataItem("Y1") == 1f));
                    main.addData("Controller 1 b", (Main.GetControllerDataItem("B1") == 1f));
                    main.addData("Controller 1 a", (Main.GetControllerDataItem("A1") == 1f));
                    try {
                        Thread.sleep(100);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                }
            }


        }
    }

    private static volatile HashMap<String, Float> ControllerData = new HashMap<String, Float>();
    //private Controls_Update ControlsUpdate = new Controls_Update();
    //private Data_Update DataUpdate = new Data_Update();
    //private Thread dta = new Thread(DataUpdate);
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Updates Data
         */
        ControllerData.put("LeftStickX1", 0.0f);
        ControllerData.put("LeftStickX2", 0.0f);
        ControllerData.put("RightStickX1", 0.0f);
        ControllerData.put("RightStickX2", 0.0f);
        ControllerData.put("X1", 0.0f);
        ControllerData.put("X2", 0.0f);
        ControllerData.put("Y1", 0.0f);
        ControllerData.put("Y2", 0.0f);
        ControllerData.put("A1", 0.0f);
        ControllerData.put("A2", 0.0f);
        ControllerData.put("B1", 0.0f);
        ControllerData.put("B2", 0.0f);
        ControllerData.put("LeftBumper1", 0.0f);
        ControllerData.put("LeftBumper2", 0.0f);
        ControllerData.put("RightBumper1", 0.0f);
        ControllerData.put("RightBumper2", 0.0f);
        ControllerData.put("LeftTrigger1", 0.0f);
        ControllerData.put("LeftTrigger2", 0.0f);
        ControllerData.put("RightTrigger1", 0.0f);
        ControllerData.put("RightTrigger2", 0.0f);
        waitForStart();
        //ControlsUpdate.runOpMode();
        //dta.start();
        if(opModeIsActive()){
            while(opModeIsActive()){
                //telemetry.addData("Test", true);
                telemetry.update();
            }
            //DataUpdate.active = false;
        }
    }
    /**
     * used to get controller data
     * @param key the key for the item you want to get
     * @return the data of the requested item
     * */
    public static float GetControllerDataItem(String key){
        return ControllerData.get(key);
    }
    /**
     * used for updating the controller data
     * @param key the key for the item that you want to change
     * @param data the data that you want to store
     * */
    public static void updateControllerDataItem(String key, Float data) {
        ControllerData.replace(key, data);
    }
    public void addData(String name, float data){
        telemetry.addData(name, data);
    }
    public void addData(String name, boolean data){
        telemetry.addData(name, data);
    }
    public void addData(String name, String data){
        telemetry.addData(name, data);
    }
    public void updateData(){
        addData("Controller 1 left stick", (Main.GetControllerDataItem("LeftStickX1") + "," + Main.GetControllerDataItem("LeftStickY1")));
        addData("Controller 1 right stick", (Main.GetControllerDataItem("RightStickX1") + "," + Main.GetControllerDataItem("RightStickY1")));
        addData("Controller 1 left trigger", (float)Main.GetControllerDataItem("LeftTrigger1"));
        addData("Controller 1 right trigger", (float)Main.GetControllerDataItem("RightTrigger1"));
        addData("Controller 1 left bumper", (Main.GetControllerDataItem("LeftBumper1") == 1f));
        addData("Controller 1 right bumper", (Main.GetControllerDataItem("RightBumper1") == 1f));
        addData("Controller 1 x", (Main.GetControllerDataItem("X1") == 1f));
        addData("Controller 1 y", (Main.GetControllerDataItem("Y1") == 1f));
        addData("Controller 1 b", (Main.GetControllerDataItem("B1") == 1f));
        addData("Controller 1 a", (Main.GetControllerDataItem("A1") == 1f));
    }
}
