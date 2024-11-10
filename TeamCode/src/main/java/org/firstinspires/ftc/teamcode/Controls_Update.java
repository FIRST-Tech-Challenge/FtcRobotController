package org.firstinspires.ftc.teamcode;



import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Controls_Update extends LinearOpMode {
    //public volatile double[] left_stick_x = {0,0};
    public volatile double[] left_stick_y = {0,0};
    public volatile double[] right_stick_x = {0,0};
    public volatile double[] right_stick_y = {0,0};
    public volatile boolean[] x = {false, false};
    public volatile boolean[] y = {false, false};
    public volatile boolean[] b = {false, false};
    public volatile boolean[] a = {false, false};
    public volatile double[] right_bumper = {0,0};
    public volatile double[] left_bumper = {0,0};
    public volatile double[] right_trigger = {0,0};
    public volatile double[] left_trigger = {0,0};
    public volatile Gamepad gamepad1 = null;
    public volatile Gamepad gamepad2 = null;
    public volatile boolean active = true;
    @Override
    public void runOpMode() throws InterruptedException {
        if(active){
            while(active) {
                Main.updateControllerDataItem("LeftStickX1", gamepad1.left_stick_x);
                Main.updateControllerDataItem("LeftStickX2", gamepad2.left_stick_x);
                Main.updateControllerDataItem("LeftStickY1", gamepad1.left_stick_y);
                Main.updateControllerDataItem("LeftStickY2", gamepad2.left_stick_y);
                Main.updateControllerDataItem("RightStickY1", gamepad1.right_stick_y);
                Main.updateControllerDataItem("RightStickY2", gamepad2.right_stick_y);
                Main.updateControllerDataItem("RightStickX1", gamepad1.right_stick_x);
                Main.updateControllerDataItem("RightStickX2", gamepad2.right_stick_x);
                Main.updateControllerDataItem("X1", (gamepad1.x ? 1f : 0f));
                Main.updateControllerDataItem("X2", (gamepad2.x ? 1f : 0f));
                Main.updateControllerDataItem("Y1", (gamepad1.y ? 1f : 0f));
                Main.updateControllerDataItem("Y2", (gamepad2.y ? 1f : 0f));
                Main.updateControllerDataItem("A1", (gamepad1.a ? 1f : 0f));
                Main.updateControllerDataItem("A1", (gamepad2.a ? 1f : 0f));
                Main.updateControllerDataItem("B1", (gamepad1.b ? 1f : 0f));
                Main.updateControllerDataItem("B2", (gamepad2.b ? 1f : 0f));
                Main.updateControllerDataItem("RightBumper1", (gamepad1.right_bumper ? 1f : 0f));
                Main.updateControllerDataItem("RightBumper2", (gamepad2.right_bumper ? 1f : 0f));
                Main.updateControllerDataItem("LeftBumper1", (gamepad1.left_bumper ? 1f : 0f));
                Main.updateControllerDataItem("LeftBumper2", (gamepad2.left_bumper ? 1f : 0f));
                Main.updateControllerDataItem("RightTrigger1", gamepad1.right_trigger);
                Main.updateControllerDataItem("RightTrigger2", gamepad2.right_trigger);
                Main.updateControllerDataItem("LeftTrigger1", gamepad1.left_trigger);
                Main.updateControllerDataItem("LeftTrigger2", gamepad2.left_trigger);
            }
        }

    }
}
