
package autofunctions;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.util.ArrayList;

import global.TerraBot;
import util.Rect;

public class TerraCV {
//
//    public Helper h = new Helper();
//    public TerraBot bot;
//    public VuforiaLocalizer vuforia;
//    public VuforiaLocalizer.CloseableFrame currentFrame;
//    public Image img;
//    public Bitmap bm;
//    LinearOpMode op;
//
//    public boolean discount = false;
//
//    public int Accuracy = 7;
//
//
//    ElapsedTime debug = new ElapsedTime();
//    public double time = 0;
//
//
//    public void init(TerraBot b, LinearOpMode o, int acc){
//        bot = b;
//        op = o;
//        Accuracy = acc;
//        int cameraMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
//        //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//        parameters.vuforiaLicenseKey = h.VUFORIA_KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//
//       vuforia = ClassFactory.getInstance().createVuforia(parameters);
//       //CameraDevice.getInstance().setFlashTorchMode(true);
//
//       Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
//       vuforia.setFrameQueueCapacity(1);
//
//    }
//    public void takePicture(){
//        //1280, 720
//        while (op.opModeIsActive() && img == null) {
//            try {currentFrame = vuforia.getFrameQueue().take();}catch (InterruptedException e){}
//            long numImages = currentFrame.getNumImages();
//            for (int i = 0; i < numImages; i++) {
//                if (currentFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    img = currentFrame.getImage(i);
//                    break;
//                }
//            }
//            if (img != null) {
//                bm = vuforia.convertFrameToBitmap(currentFrame);
//            }
//        }
//    }
//    public void takePictureBeforeInit(){
//        //1280, 720
//        resetImg();
//        while (!op.isStarted() && img == null) {
//            try {currentFrame = vuforia.getFrameQueue().take();}catch (InterruptedException e){}
//            long numImages = currentFrame.getNumImages();
//            for (int i = 0; i < numImages; i++) {
//                if (currentFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
//                    img = currentFrame.getImage(i);
//                    break;
//                }
//            }
//            if (img != null) {
//                bm = vuforia.convertFrameToBitmap(currentFrame);
//            }
//        }
//    }
//    public void resetImg(){
//        img = null;
//    }
//    public StonePos getStonePos(Rect area){
//        debug.reset();
//        StonePos pos;
//        int th = area.getWidth() / 3;
//        Rect left = new Rect(area.getX1(), area.getY1(), th, area.getHeight());
//        Rect middle = new Rect(area.getX1() + th, area.getY1(), th, area.getHeight());
//        Rect right = new Rect(area.getX1() + 2 * th, area.getY1(), th, area.getHeight());
//        double[] values = new double[3];
//        values[0] = getAverageOfPixelsBeforeInit(left);
//        values[1] = getAverageOfPixelsBeforeInit(middle);
//        values[2] = getAverageOfPixelsBeforeInit(right);
//        int index = h.findMin(values);
//        if (index == 0) {
//            pos = StonePos.LEFT;
//        } else if (index == 1) {
//            pos = StonePos.MIDDLE;
//        } else {
//            pos = StonePos.RIGHT;
//        }
//        time = debug.milliseconds();
//
//        if(!discount) {
//            return pos;
//        }else{
//            return null;
//        }
//    }
//    public double getAverageOfPixelsBeforeInit(Rect rect){
//        double total = 0;
//        int x1 = rect.getX1();
//        int y1 = rect.getY1();
//        int x2 = rect.getX2();
//        int y2 = rect.getY2();
//
//        for (int x = x1; x < x2; x+= Accuracy) {
//            for (int y = y1; y < y2; y+=Accuracy ) {
//                int pix = bm.getPixel(x,y);
//                float[] hsv = h.rgbToHSV(pix);
//                total += hsv[2];
//                if(op.isStarted()){
//                    break;
//                }
//            }
//            if(op.isStarted()){
//                discount = true;
//                break;
//            }
//        }
//        return total/(rect.getArea());
//    }
//    public enum StonePos{
//        LEFT,
//        RIGHT,
//        MIDDLE
//    }
}