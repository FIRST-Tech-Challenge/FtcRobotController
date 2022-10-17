package org.firstinspires.ftc.teamcode;

import static android.graphics.Color.blue;
import static android.graphics.Color.green;
import static android.graphics.Color.red;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.Parameters;


public class Vision {
    private LinearOpMode opMode;

    private VuforiaLocalizer vuforia;
    private Parameters parameters;
    private CameraDirection CAMERA_CHOICE = CameraDirection.BACK; // This is the webcam.
    private static final boolean PHONE_IS_PORTRAIT = false;
    private static final String VUFORIA_KEY = "AcELeNr/////AAABmeg7NUNcDkPigDGNImdu5slLREdKn/q+qfajHBypycR0JUZYbfU0q2yZeSud79LJ2DS9uhr7Gu0xDM0DQZ36GRQDgMRwB8lf9TGZFQcoHq4kVAjAoEByEorXCzQ54ITCextAucpL2njKT/1IJxgREr6/axNEL2evyKSpOKoNOISKR6tkP6H3Ygd+FHm2tF/rsUCJHN5bTXrbRbwt5t65O7oJ6Wm8Foz1npbFI0bsD60cug4CpC/Ovovt2usxIRG8cpoQX49eA2jPRRLGXN8y1Nhh9Flr0poOkYoCExWo2iVunAGOwuCdB/rp/+2rkLBfWPvzQzrN9yBBP0JVJZ4biNQ41qqiuVvlc31O9xEvbKHt";

    private final int RED_LOW = 165; //i guess we can steal this for orange detection
    private final int GREEN_LOW = 74;
    private final int BLUE_LOW = 4;
    private final int RED_HIGH = 197;
    private final int GREEN_HIGH = 107;
    private final int BLUE_HIGH = 54;

    private final int CYAN_RED_LOW = 0;
    private final int CYAN_RED_HIGH = 75;
    private final int CYAN_BlUE_LOW = 225; //no need for cyan blue high, because the high is 255
    private final int CYAN_BlUE_HIGH = 255;
    private final int CYAN_GREEN_LOW = 225; //same reasoning
    private final int CYAN_GREEN_HIGH = 255; //same reasoning

    private final int MAGENTA_RED_LOW = 225;
    private final int MAGENTA_RED_HIGH = 255;
    private final int MAGENTA_BLUE_LOW = 225; //i am assuming the same holds for magenta
    private final int MAGENTA_BLUE_HIGH = 255; //i am assuming the same holds for magenta
    private final int MAGENTA_GREEN_LOW = 0;
    private final int MAGENTA_GREEN_HIGH = 75;


    public Vision(LinearOpMode opMode, char side) {

        this.opMode = opMode;

        int cameraMonitorViewId = this.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", this.opMode.hardwareMap.appContext.getPackageName());
        Parameters params = new Parameters(cameraMonitorViewId);

        params.vuforiaLicenseKey = VUFORIA_KEY;
        if (side == 'r'){
            params.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam");
        }
        else{
            //whatever
        }


        vuforia = ClassFactory.getInstance().createVuforia(params);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); // Format returns 2 bytes per pixel in GGGBBBBB RRRRRGGG format (little-endian)
        vuforia.setFrameQueueCapacity(4);
        vuforia.enableConvertFrameToBitmap();

    }

    public int levelIdent(char side) {
        Bitmap bitmap = null;
        try {
            bitmap = getBitmap();
        } catch (InterruptedException e) {
            opMode.telemetry.addData("ERROR: ", "Cringemac ate lao gan ma and then aggressively started pooping his pants");
            opMode.telemetry.update();
        }

        int level = 3;

        int xherny; //x
        int yherny; //y
        int crongemac; //x-val iterator
        int monkus = 1; //y-val iterator
        xherny = bitmap.getWidth() - 1;
        yherny = bitmap.getHeight() - 1;

        while (!isMagenta(bitmap.getPixel(xherny, yherny)) && !isCyan(bitmap.getPixel(xherny, yherny)) && !isOrange(bitmap.getPixel(xherny, yherny)) && monkus < bitmap.getHeight()) {
            yherny = bitmap.getHeight() - monkus;
            crongemac = 1;
            xherny = bitmap.getWidth() - crongemac;
            while (!isOrange(bitmap.getPixel(xherny, yherny)) && crongemac < 465) {
                xherny = bitmap.getWidth() - crongemac;
                crongemac++;
            }
            monkus++;
            if (monkus >= 470){
                level = 3;
            }
        }

        if (isMagenta(bitmap.getPixel(xherny, yherny))) {
            level = 1;
        }
        else if (isCyan(bitmap.getPixel(xherny, yherny))) {
            level = 2;
        }
        else {
            level = 3;
        }

        return level;
    }

    private boolean isCyan(int pixel) {
        return ((CYAN_RED_LOW <= red(pixel)) && (red(pixel) <= CYAN_RED_HIGH) && (CYAN_GREEN_LOW <= green(pixel)) &&
                    (green(pixel) <= CYAN_GREEN_HIGH) && (CYAN_BlUE_LOW<= blue(pixel)) && (blue(pixel) <= CYAN_BlUE_HIGH));
    }

    private boolean isOrange(int pixel) {
        return ((RED_LOW <= red(pixel)) && (red(pixel) <= RED_HIGH) && (GREEN_LOW <= green(pixel)) &&
                (green(pixel) <= GREEN_HIGH) && (BLUE_LOW <= blue(pixel)) && (blue(pixel) <= BLUE_HIGH));
    }


    public boolean isMagenta(int pixel){
        return ((MAGENTA_RED_LOW <= red(pixel)) && (red(pixel) <= MAGENTA_RED_HIGH) && (MAGENTA_GREEN_LOW <= green(pixel)) &&
                (green(pixel) <= MAGENTA_GREEN_HIGH) && (MAGENTA_BLUE_LOW<= blue(pixel)) && (blue(pixel) <= MAGENTA_BLUE_HIGH));
    }

    public Bitmap getBitmap() throws InterruptedException {

        VuforiaLocalizer.CloseableFrame picture;
        picture = vuforia.getFrameQueue().take();
        Image rgb = picture.getImage(1);

        long numImages = picture.getNumImages();

        for (int i = 0; i < numImages; i++) {

            int format = picture.getImage(i).getFormat();
            if (format == PIXEL_FORMAT.RGB565) {
                rgb = picture.getImage(i);
                break;
            }
        }

        Bitmap imageBitmap = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
        imageBitmap.copyPixelsFromBuffer(rgb.getPixels());

        picture.close();

        return imageBitmap;
    }
}