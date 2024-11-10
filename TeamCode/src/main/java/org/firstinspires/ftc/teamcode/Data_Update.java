package org.firstinspires.ftc.teamcode;

public class Data_Update extends Thread {

    public volatile boolean active = true;
    Main main = new Main();
    public void run() {
        if(active){
            while(active){
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
