
package autofunctions;

import android.graphics.Bitmap;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.CameraDevice;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

import util.Rect;

public class TerraCV {

    public VuforiaLocalizer vuforia;
    public final String VUFORIA_KEY ="AdfjEqf/////AAABmUFlTr2/r0XAj6nkD8iAbHMf9l6LwV12Hw/ie9OuGUT4yTUjukPdz9SlCFs4axhhmCgHvzOeNhrjwoIbSCn0kCWxpfHAV9kakdMwFr6ysGpuQ9xh2xlICm2jXxVfqYKGlWm3IFk1GuGR7N5jt071axc/xFBQ0CntpghV6siUTyuD4du5rKhqO1pp4hILhJLF5I6LbkiXN93utfwje/8kEB3+V4TI+/rVj9W+c7z26rAQ34URhQ5AcPlhIfjLyUcTW15+UylM0dxGiMpQprreFVaOk32O2epod9yIB5zgSin1bd7PiCXHbPxhVhMz0cMNRJY1LLfuDru3npuemePUkpSOp5SFbuGjzso9hDA/6V3L";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public TFObjectDetector tfod;
    public double minConf = 0.7;


    public VuforiaLocalizer.CloseableFrame currentFrame;
    public Image img;
    public Bitmap bm;

    public boolean discount = false;

    public int accuracy = 3;


    ElapsedTime debug = new ElapsedTime();
    public double time = 0;

    LinearOpMode op;
    public void init(LinearOpMode op, boolean visible){
        this.op = op;
        initVuforia();
        initTF(visible);
    }
    public void initVuforia(){
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);
        CameraDevice.getInstance().setFlashTorchMode(true);
    }
    public void initTF(boolean visible){
        TFObjectDetector.Parameters tfodParameters;
        if(visible) {
            int tfodMonitorViewId = op.hardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", op.hardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }else{
            tfodParameters = new TFObjectDetector.Parameters();
        }
        tfodParameters.minResultConfidence = (float) minConf;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
//        tfod.setZoom(2.5, 1.78);

    }
//
//    public RingNum scanRingsBeforeInit(){
//        RingNum out = RingNum.ZERO;
//        tfod.activate();
//        //tfod.setClippingMargins(100,500,400,200);
//        //tfod.setZoom(2.5, 1.78);
//        while (!op.isStarted() && !op.isStopRequested()){
//            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
//            if (updatedRecognitions != null) {
//                for (Recognition recognition : updatedRecognitions) {
//                    op.telemetry.addData("label", recognition.getLabel());
//                    op.telemetry.update();
//                    if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
//                        out = RingNum.ONE;
//                    }else if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
//                        out = RingNum.FOUR;
//                    }
//                }
//            }
//            op.idle();
//        }
//        tfod.shutdown();
//        return out;
//    }


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
    public void takePictureBeforeInit(){
        //1280, 720
        resetImg();
        while (!op.isStarted() && img == null && !op.isStopRequested()) {
            try {currentFrame = vuforia.getFrameQueue().take();}catch (InterruptedException e){}
            long numImages = currentFrame.getNumImages();
            for (int i = 0; i < numImages; i++) {
                if (currentFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                    img = currentFrame.getImage(i);
                    break;
                }
            }
            if (img != null) {
                bm = vuforia.convertFrameToBitmap(currentFrame);
            }
        }
    }
    public void resetImg(){
        img = null;
    }
    public RingNum getRingNum(Rect area){
        debug.reset();
        RingNum num = RingNum.ZERO;
        double val = 0;
        time = debug.milliseconds();
//        op.telemetry.addData("val", val);
//        op.telemetry.update();

        double max = 0;
        double enx = 0;
        double eny = 0;
        accuracy = 5;
        for (int x = 400; x < 900; x+=50) {
            for (int y = 0; y < 620; y+=50) {
                Rect ar = new Rect(x,y,100,100);
                val = getAverageOfPixelsBeforeInit(ar)*1000;
                if(val > max){
                    max = val;
                    enx = x;
                    eny = y;
                }
            }
        }
        if(600 < enx && enx < 800 && 400 < eny && eny < 700){
            if (max > 25) {
                num = RingNum.FOUR;
            } else if (max > 10) {
                num = RingNum.ONE;
            }
        }

        op.telemetry.addData("val", max);
        op.telemetry.addData("enx", enx);
        op.telemetry.addData("eny", eny);
        op.telemetry.addData("num", num.toString());
        op.telemetry.update();

        if(!discount) {
            return num;
        }else{
            return null;
        }
    }
    public double getAverageOfPixelsBeforeInit(Rect rect){
        double total = 0;
        int x1 = rect.getX1();
        int y1 = rect.getY1();
        int x2 = rect.getX2();
        int y2 = rect.getY2();

        int[] max = new int[]{0,0};

        for (int x = x1; x < x2; x+= accuracy) {
            for (int y = y1; y < y2; y+= accuracy ) {
                int pix = bm.getPixel(x,y);
                float[] hsv = rgbToHSV(pix);
//                if(35 < hsv[0] && hsv[0] < 65) {
//                    total += hsv[1];
//                }
                if(20 < hsv[0] && hsv[0] < 50) {
                    total += hsv[1];
                }
                //total += hsv[2];
                if(op.isStarted() || op.isStopRequested()){
                    break;
                }
            }
            if(op.isStarted() || op.isStopRequested()){
                discount = true;
                break;
            }
        }
//
//        int pix = bm.getPixel(600,350);
//        float[] hsv = rgbToHSV(pix);
//        op.telemetry.addData("0", hsv[0]);
//        op.telemetry.addData("1", hsv[1]);
//        op.telemetry.addData("2", hsv[2]);
//        op.telemetry.update();
//        op.telemetry.addData("posx", max[0]);
//        op.telemetry.addData("posy", max[1]);
//        op.telemetry.update();
        return total/(rect.getArea());
    }
    public float[] rgbToHSV(int pix){
        int r = Color.red(pix);
        int g = Color.green(pix);
        int b = Color.blue(pix);
        float[] hsv = new float[3];
        Color.RGBToHSV(r, g, b, hsv);
        return hsv;
    }
    public enum RingNum {
        ZERO,
        ONE,
        FOUR
    }
}