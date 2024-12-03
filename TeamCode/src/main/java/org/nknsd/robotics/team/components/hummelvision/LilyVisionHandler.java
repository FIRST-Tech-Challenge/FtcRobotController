package org.nknsd.robotics.team.components.hummelvision;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.nknsd.robotics.framework.NKNComponent;


public class LilyVisionHandler implements NKNComponent {

    public static class VisionData{
        // values will equal Integer.MAX_VALUE
        // if not sensed for that color/dimension
        public final int yellowX,yellowY;
        public final int redX,redY;
        public final int blueX,blueY;

        public VisionData(int yellowX, int yellowY, int redX, int redY, int blueX, int blueY) {
            this.yellowX = yellowX;
            this.yellowY = yellowY;
            this.redX = redX;
            this.redY = redY;
            this.blueX = blueX;
            this.blueY = blueY;
        }
    }

    private LilyI2cDevice visionDevice;

    @Override
    public boolean init(Telemetry telemetry, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        visionDevice = hardwareMap.get(LilyI2cDevice.class, "vision_sensor");
        return true;
    }

    @Override
    public void init_loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void start(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void stop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public String getName() {
        return null;
    }

    @Override
    public void loop(ElapsedTime runtime, Telemetry telemetry) {

    }

    @Override
    public void doTelemetry(Telemetry telemetry) {
        VisionData data = getVisionData();
        String output = "Y[";
        if (data.yellowX != Integer.MAX_VALUE){
            output += data.yellowX+","+data.yellowY + "]";
        } else {
            output += "NONE]";
        }
        output += " R[";
        if (data.redX != Integer.MAX_VALUE){
            output += data.redX+","+data.redY + "]";
        } else {
            output += "NONE]";
        }
        output += " B[";
        if (data.blueX != Integer.MAX_VALUE){
            output += data.blueX+","+data.blueY + "]";
        } else {
            output += "NONE]";
        }


        telemetry.addData("vision",output);
    }

    public VisionData getVisionData(){
        int[] data = visionDevice.getData();
        int yellowX = Integer.MAX_VALUE;
        int yellowY = Integer.MAX_VALUE;
        int redX = Integer.MAX_VALUE;
        int redY = Integer.MAX_VALUE;
        int blueX = Integer.MAX_VALUE;
        int blueY = Integer.MAX_VALUE;

        if(data[0] < 255){
            yellowX = -data[0];
        } else if (data[1] < 255){
            yellowX = data[1];
        }

        if(data[2] < 255){
            yellowY = -data[2];
        } else if (data[3] < 255){
            yellowY = data[3];
        }

        if(data[4] < 255){
            redX = -data[4];
        } else if (data[5] < 255){
            redX = data[5];
        }

        if(data[6] < 255){
            redY = -data[6];
        } else if (data[7] < 255){
            redY = data[7];
        }

        if(data[8] < 255){
            blueX = -data[8];
        } else if (data[9] < 255){
            blueX = data[9];
        }

        if(data[10] < 255){
            blueY = -data[10];
        } else if (data[11] < 255){
            blueY = data[11];
        }



        return new VisionData(yellowX, yellowY, redX,
                redY, blueX, blueY);
    }
}
