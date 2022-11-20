package org.firstinspires.ftc.teamcode;


import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Log;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.code.Attribute;

import java.util.*;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.android.dx.util.Warning;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;
import org.firstinspires.ftc.teamcode.Constants;

/**
 * Detector class that uses the Vuforia engine to grab a bitmap from the phone camera and detect the location of the Team Marker.
 */
public class TeamMarkerDetector {
  private static final String VUFORIA_KEY = "ASxSfhX/////AAABmWcpvgdyP053gmhPvX7/JZ5yybQKAVFnqMk+WjYvbuuiectzmcdkuftxSIgVawrOZ7CQOqdHzISXbHCAom4FhIzrDceJIIEGozFWpgAu5dUKc3q843Hd3x875VOBf8B7DlD7g9TgqxqgQRw9coEUBBeEJqy2KGy4NLPoIKLdiIx8yxSWm7SlooFSgmrutF/roBtVM/N+FhY6Sgdy9fgWssccAhd2IxdYllAaw4s1oC1jqtwbjIsdjNVogmwwXdTmqiKHait1PFyF2FDNfKi+7qs4Mc6KbvXD2FHA6RljkcN5Oo080o2QSVCzDuQtJeagh/CglB2PcatFWnebiWN+a43kEdrUaY+uq0YQ8m9IRBWE";

  private VuforiaLocalizer vuforiaLocalizer;


  private static final int Y_COORD = 955;

  private int stoneRightX, stoneCenterX, stoneLeftX;

  private static final int STONE_WIDTH = 340, STONE_HEIGHT = 460;

  private static final ColorPreset ACTIVE_YELLOW = ColorPreset.PURE_YELLOW;
  private static final ColorPreset ACTIVE_BLACK = ColorPreset.PURE_GRAY;

  private enum ColorPreset {
    PURE_YELLOW(255, 255, 0),
    PURE_GRAY(128, 123, 125);

    int r, g, b;

    ColorPreset(int r, int g, int b) {
      this.r = r;
      this.g = g;
      this.b = b;
    }
  }

  public TeamMarkerDetector(int cameraMonitorViewId) {
    initVuforia(cameraMonitorViewId);
  }

  /**
   * Runs the image processing code.
   *
   * @return The location of the team marker; LEFT, CENTER, or RIGHT
   */
  public Constants.SamplingLocation sample(boolean imageSavingEnabled) {


//look up to location
    // THE PICTURE IS 1280x720 PLAESLKHDFKJADSHFKJSDHFKLHDFSKHSDKHDSFKHHS
    Bitmap vuBitmap = getBitmap();
    if (imageSavingEnabled) {
      FileUtils.saveImage(vuBitmap, null);
    }

    Log.println(Log.INFO, "bitmapFILTER", String.valueOf(vuBitmap.getWidth()));
    Log.println(Log.INFO, "bitmapFILTER", String.valueOf(vuBitmap.getHeight()));

    try {
      Thread.sleep(1000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }



    Bitmap stoneRight = Bitmap.createBitmap(vuBitmap, 1026, 269, 175, 175);
    Log.println(Log.INFO, "bitmapFILTER completed the right", String.valueOf(vuBitmap.getHeight()));
    Bitmap stoneCenter = Bitmap.createBitmap(vuBitmap, 600, 269, 175, 175);
    Log.println(Log.INFO, "bitmapFILTER completed the middle", String.valueOf(vuBitmap.getHeight()));
    Bitmap stoneLeft = Bitmap.createBitmap(vuBitmap, 190, 269, 175, 175);
    Log.println(Log.INFO, "bitmapFILTER completed the left", String.valueOf(vuBitmap.getHeight()));


    if (imageSavingEnabled) {
      FileUtils.saveImage(stoneRight, Constants.SamplingLocation.RIGHT);
      FileUtils.saveImage(stoneCenter, Constants.SamplingLocation.CENTER);
      FileUtils.saveImage(stoneLeft, Constants.SamplingLocation.LEFT);
    }

    // Ratio is measured blackness to yellowness. higher ratio is more likeliness to be a team marker.
//    double ratioRight = getClosenessToColor(stoneRight, ACTIVE_BLACK) / getClosenessToColor(stoneRight, ACTIVE_YELLOW);
//    double ratioCenter = getClosenessToColor(stoneCenter, ACTIVE_BLACK) / getClosenessToColor(stoneCenter, ACTIVE_YELLOW);
//    double ratioLeft = getClosenessToColor(stoneLeft, ACTIVE_BLACK) / getClosenessToColor(stoneLeft, ACTIVE_YELLOW);

    Log.println(Log.INFO, "bitmapFILTER yellow for right", String.valueOf(getClosenessToColor(stoneRight, ACTIVE_YELLOW)));
    Log.println(Log.INFO, "bitmapFILTER yellow for center", String.valueOf(getClosenessToColor(stoneCenter, ACTIVE_YELLOW)));
    Log.println(Log.INFO, "bitmapFILTER yellow for left", String.valueOf(getClosenessToColor(stoneLeft, ACTIVE_YELLOW)));
    Log.println(Log.INFO, "bitmapFILTER black for right", String.valueOf(getClosenessToColor(stoneRight, ACTIVE_BLACK)));
    Log.println(Log.INFO, "bitmapFILTER black for center", String.valueOf(getClosenessToColor(stoneCenter, ACTIVE_BLACK)));
    Log.println(Log.INFO, "bitmapFILTER black for left", String.valueOf(getClosenessToColor(stoneLeft, ACTIVE_BLACK)));


    if(getClosenessToColor(stoneLeft, ACTIVE_YELLOW) > getClosenessToColor(stoneRight, ACTIVE_YELLOW) && getClosenessToColor(stoneLeft, ACTIVE_YELLOW) > getClosenessToColor(stoneCenter, ACTIVE_YELLOW)) {
      return Constants.SamplingLocation.LEFT;
    } else if(getClosenessToColor(stoneCenter, ACTIVE_YELLOW) > getClosenessToColor(stoneLeft, ACTIVE_YELLOW) && getClosenessToColor(stoneCenter, ACTIVE_YELLOW) > getClosenessToColor(stoneRight, ACTIVE_YELLOW)) {
      return Constants.SamplingLocation.CENTER;
    } else {
      return Constants.SamplingLocation.RIGHT;
    }

//    if (ratioRight > ratioCenter && ratioRight > ratioLeft) {
//      return Constants.SamplingLocation.RIGHT;
//    } else if (ratioCenter > ratioRight && ratioCenter > ratioLeft) {
//      return Constants.SamplingLocation.CENTER;
//    } else {
//      return Constants.SamplingLocation.LEFT;
//    }
  }

  /**
   * Grabs a bitmap from the Vuforia engine for image processing
   * @return camera output as a bitmap
   */
  public Bitmap getBitmap(){
    Bitmap bitmap;
    VuforiaLocalizer.CloseableFrame frame;

    try {
      frame = vuforiaLocalizer.getFrameQueue().take();
    }catch (Exception e) {
      throw new Warning("couldn't find vuforia frame");
    }

    bitmap = vuforiaLocalizer.convertFrameToBitmap(frame);
    return bitmap;
  }

  /**
   * @return the closeness of a region to a color
   */
  private double getClosenessToColor(Bitmap bitmap, ColorPreset colorPreset) {
    int color;
    int r, g, b;
    double distanceSum = 0;

    int pixels = bitmap.getWidth() * bitmap.getHeight();
    for (int i = 0; i < bitmap.getWidth(); i++) {
      for (int j = 0; j < bitmap.getHeight(); j++) {
        color = bitmap.getPixel(i, j);
        r = Color.red(color);
        g = Color.green(color);
        b = Color.blue(color);
        distanceSum += getColorDistance(r, g, b, colorPreset.r, colorPreset.g, colorPreset.b);
      }
    }

    double averageDistance = distanceSum / pixels;

    if (averageDistance != 0) {
      return 1 / averageDistance;
    } else {
      return Double.POSITIVE_INFINITY;
    }
  }

  /**
   * Gets the distance between two colors via 3 dimensional distance formula.
   *
   * @param r       Actual red value
   * @param g       Actual green value
   * @param b       Actual blue value
   * @param targetR Target red value
   * @param targetG Target green value
   * @param targetB Target blue value
   * @return Distance between actual color and target color
   */
  private double getColorDistance(int r, int g, int b, int targetR, int targetG, int targetB) { // does the actual Maths
    int rDifference = r - targetR;
    int gDifference = g - targetG;
    int bDifference = b - targetB;

    int rDifferenceSquared = (int) Math.pow(rDifference, 2);
    int gDifferenceSquared = (int) Math.pow(gDifference, 2);
    int bDifferenceSquared = (int) Math.pow(bDifference, 2);

    int sum = rDifferenceSquared + gDifferenceSquared + bDifferenceSquared;

    return Math.sqrt(sum);
  }

  private void initVuforia(int cameraMonitorViewId) {
    VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters();//cameraMonitorViewId);
    params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
    params.vuforiaLicenseKey = VUFORIA_KEY;

    this.vuforiaLocalizer = ClassFactory.getInstance().createVuforia(params);
    vuforiaLocalizer.enableConvertFrameToBitmap();
    vuforiaLocalizer.setFrameQueueCapacity(1);
  }
}
