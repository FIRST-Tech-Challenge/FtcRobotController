package com.technototes.library.vision;


import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;


import java.util.ArrayList;

import static com.technototes.library.util.MathUtils.constrain;


public class Vision implements Runnable{
    public static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = VuforiaLocalizer.CameraDirection.FRONT;
    public static final boolean PHONE_IS_PORTRAIT = true;
    private static final String VUFORIA_KEY =
            "AX76a6T/////AAABmZU2tJ9jdUCdhRKzbUVAd/MCbSt73wl/qTBBrkwbMoAmX27XgROvE/abAz7wLqYrhdvrU7rQM4T3jprBs9uy1hSfvVVEWmwV6a0NchoYQLsVLpVzjF5G5N0VBN2aHLw5klCeT+5ZOeFhBrmlv0l/kajhYGTX3zM4FKRpdpFGqsdSX7QVcY73ay9VaGmwbejwbnwQ60qmg47t884/UE7PNxdzpR+2XV+RBvBXDng/R5fLj1A2DpkrdBDfLjS1wHb4EvJTcu065t0imRwDtpr0iLRbZrg4gjzlb4m4tq4qVU6mzib4o3kz/qHjqOqPqvMkMXFfbUWWABaMcyCLHbNLopb4uWxd1OhRUhq1p35Oe0HZ";
    private VuforiaLocalizer vuforia = null;

    public Bitmap image = null;
    public Thread t;

    public Node tlnode, trnode, blnode, brnode;

    public Vision(HardwareMap hardwareMap, Telemetry tel) {

        // Configure Vuforia
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        // Enable the image acquisition stuff (the only thing we're using)
        vuforia.enableConvertFrameToBitmap();
        t = new Thread(this);
        t.start();
    }
    public void stop(){
        t.stop();
    }

    public ArrayList<Node> topNodes = new ArrayList<>();
    public ArrayList<Node> bottomNodes = new ArrayList<>();
    long lastTime = System.currentTimeMillis();

    public void makeChoices(){
        int width = image.getWidth();
        int height = image.getHeight();
        if(topNodes.size() < 2 && bottomNodes.size() < 2){
            for(int i = 0; i < width; i+=4){
                for(int j = 0; j < height; j+=4) {
                    int p1 = image.getPixel(i, j);
                    float[] hsv1 = new float[3];
                    Color.colorToHSV(p1, hsv1);
                    int h1 = (int) (hsv1[0] * 360);
                    int s1 = (int) (hsv1[1] * 100);
                    int v1 = (int) (hsv1[2] * 100);
                    if ((h1 > 340 || h1 < 11) && v1 > 30 && s1 > 30) {
                        int topBlue = 0, topRed = 0, bottomBlue = 0, bottomRed = 0;
                        for(int x = constrain(0, i-10, width-1); x < constrain(0, i+10, width-1); x+=1){
                            for(int y = constrain(0, j-20, height-1); y < constrain(0, j+20, height-1); y+=1){

                                int p = image.getPixel(x, y);
                                float[] hsv = new float[3];
                                Color.colorToHSV(p, hsv);
                                int h = (int) (hsv[0] * 360);
                                int s = (int) (hsv[1] * 100);
                                int v = (int) (hsv[2] * 100);
                                if((h > 200 && h < 260) && v > 15 && s > 30){
                                    //System.out.println("e");
                                    if(y < j){
                                        topBlue++;
                                    } else{
                                        bottomBlue++;
                                    }
                                }
                                if((h > 340 || h < 11) && v > 30 && s > 30){
                                    //System.out.println("eeeee");
                                    if(y < j){
                                        topRed++;
                                    } else{
                                        bottomRed++;
                                    }
                                }
                            }

                        }
                        boolean b = true;
                        for(Node n : topNodes){
                            if(n.isClose(i, j)){
                                b = false;
                            }
                        }
                        for(Node n : bottomNodes){
                            if(n.isClose(i, j)){
                                b = false;
                            }
                        }
                        if(b) {
                            if (topBlue > 20 && bottomRed > 20) {
                                //System.out.println("top");
                                topNodes.add(new Node(i, j).setRatings(bottomRed, topBlue));
                            } else if (topRed > 20 && bottomBlue > 20) {
                                //System.out.println("bottom");
                                bottomNodes.add(new Node(i, j).setRatings(topRed, bottomBlue));
                            }
                        }
                    }
                }
            }
        }

        for(int i = 0; i < topNodes.size(); i++){
            for (int j = i+1; j < topNodes.size(); j++){
                Node in = topNodes.get(i);
                Node jn = topNodes.get(j);
                if(in.isClose(jn)){
                    if(in.bluer+in.redr>jn.bluer+jn.redr){
                        topNodes.remove(jn);
                    }else{
                        topNodes.remove(in);
                    }
                    j--;
                }
            }
        }
        for(int i = 0; i < bottomNodes.size(); i++){
            for (int j = i+1; j < bottomNodes.size(); j++){
                Node in = bottomNodes.get(i);
                Node jn = bottomNodes.get(j);
                if(in.isClose(jn)){
                    if(in.bluer+in.redr>jn.bluer+jn.redr){
                        bottomNodes.remove(jn);
                    }else{
                        bottomNodes.remove(in);
                    }
                    j--;
                }
            }
        }
        Node top1 = new Node(0, 0).setRatings(0,0), top2 = new Node(0, 0).setRatings(0,0);
        Node bot1 = new Node(0, 0).setRatings(0,0), bot2 = new Node(0, 0).setRatings(0,0);
        for(Node n : topNodes){
            if(n.redr+n.bluer > top2.redr+top2.bluer){
                if(n.redr+n.bluer > top1.redr+top1.bluer){
                    top2 = top1;
                    top1 = n;
                }else{
                    top2 = n;
                }
            }

        }
        for(Node n : bottomNodes){
            if(n.redr+n.bluer > bot2.redr+bot2.bluer){
                if(n.redr+n.bluer > bot1.redr+bot1.bluer){
                    bot2 = bot1;
                    bot1 = n;
                }else{
                    bot2 = n;
                }
            }
        }

        tlnode = top1.x > top2.x ? top2 : top1;
        trnode = top1.x <= top2.x ? top2 : top1;
        blnode = bot1.x > bot2.x ? bot2 : bot1;
        brnode = bot1.x <= bot2.x ? bot2 : bot1;
        topNodes = new ArrayList<>();
        bottomNodes = new ArrayList<>();
        //System.out.println((int)(1/((System.currentTimeMillis()-lastTime)/1000.0)));
        lastTime = System.currentTimeMillis();

    }



    @Override
    public void run() {
        //TODO
    }



}
