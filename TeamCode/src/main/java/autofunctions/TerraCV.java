
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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

import global.TerraBot;
import util.Rect;

public class TerraCV {

    public VuforiaLocalizer vuforia;
    public final String VUFORIA_KEY ="AdfjEqf/////AAABmUFlTr2/r0XAj6nkD8iAbHMf9l6LwV12Hw/ie9OuGUT4yTUjukPdz9SlCFs4axhhmCgHvzOeNhrjwoIbSCn0kCWxpfHAV9kakdMwFr6ysGpuQ9xh2xlICm2jXxVfqYKGlWm3IFk1GuGR7N5jt071axc/xFBQ0CntpghV6siUTyuD4du5rKhqO1pp4hILhJLF5I6LbkiXN93utfwje/8kEB3+V4TI+/rVj9W+c7z26rAQ34URhQ5AcPlhIfjLyUcTW15+UylM0dxGiMpQprreFVaOk32O2epod9yIB5zgSin1bd7PiCXHbPxhVhMz0cMNRJY1LLfuDru3npuemePUkpSOp5SFbuGjzso9hDA/6V3L";
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    public TFObjectDetector tfod;
    public double minConf = 0.8;

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
    }

    public RINGNUM scanRingsBeforeInit(){
        RINGNUM out = RINGNUM.ZERO;
        tfod.activate();
        while (!op.isStarted()){
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    op.telemetry.addData("label", recognition.getLabel());
                    op.telemetry.update();
                    if(recognition.getLabel().equals(LABEL_FIRST_ELEMENT)){
                        out = RINGNUM.ONE;
                    }else if(recognition.getLabel().equals(LABEL_SECOND_ELEMENT)){
                        out = RINGNUM.FOUR;
                    }
                }
            }
        }
        tfod.shutdown();
        return out;
    }


//
//    public Helper h = new Helper();
//    public TerraBot bot;

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
    public enum RINGNUM{
        ZERO,
        ONE,
        FOUR
    }
}