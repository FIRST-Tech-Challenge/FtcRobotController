package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

@TeleOp
public class Main extends LinearOpMode {


    //private static volatile HashMap<String, Float> ControllerData = new HashMap<String, Float>();
    private Servo Hand_Rotator_Servo; // rotates the hand
    private CRServo Hand_Servo;// open and closes the hand
    //private Controls_Update ControlsUpdate = new Controls_Update();
    //private Data_Update DataUpdate = new Data_Update();
    //private Thread dta = new Thread(DataUpdate);
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Updates Data
         */
        /*ControllerData.put("LeftStickX1", 0.0f);
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
        ControllerData.put("RightTrigger2", 0.0f);*/
        Hand_Rotator_Servo = hardwareMap.get(Servo.class, "hand_rotator");
        Hand_Servo = hardwareMap.get(CRServo.class, "hand_servo");
        waitForStart();
        //ControlsUpdate.runOpMode();
        //dta.start();
        if(opModeIsActive()){
            while(opModeIsActive()){
                //telemetry.addData("Test", true);
                Hand_Rotator_Servo.setPosition(Hand_Rotator_Servo.getPosition() + gamepad1.left_stick_x);
                Hand_Servo.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
                telemetry.update();
                //updateData();
            }
            //DataUpdate.active = false;
        }
    }
    /**
     * used to get controller data
     * @param key the key for the item you want to get
     * @return the data of the requested item
     * */
    /*public static float GetControllerDataItem(String key) throws NullPointerException{

        return ControllerData.get(key);
    }*/
    /**
     * used for updating the controller data
     * @param key the key for the item that you want to change
     * @param data the data that you want to store
     * */
    /*public static void updateControllerDataItem(String key, Float data) {
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
    }*/
    /*public void updateData(){
        addData("test", "Test");
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
    }*/
}
