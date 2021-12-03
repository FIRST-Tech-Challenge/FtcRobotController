package org.firstinspires.ftc.teamcode.AdrianControls;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.firstinspires.ftc.teamcode.DbgLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class VuforiaStuff {

    VuforiaLocalizer vuforia;

    public VuforiaStuff(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public enum capElementPos {
        LEFT, CENTER, RIGHT;
    }

    public class capElementPositionData{
        public capElementPos capElementPosition;
        public double yellowCountLeft;
        public double yellowCountRight;
        public double yellowCountCenter;
    }
    public capElementPositionData vuforiascan(boolean saveBitmaps, boolean red) {
        Image rgbImage = null;
        int rgbTries = 0;
        /*
        double colorcountL = 0;
        double colorcountC = 0;
        double colorcountR = 0;
        */
        double yellowCountL = 1;
        double yellowCountC = 1;
        double yellowCountR = 1;
        //ForYellowDectection Thresholds
        //int redThreshold = 90;
        //int greenThreshold = 90;
        //int blueThreshold = 50;
        //ForBlueDectection Thresholds
        int redThreshold = 150;
        int greenThreshold = 150;
        int blueThreshold = 150;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        VuforiaLocalizer.CloseableFrame closeableFrame = null;
        this.vuforia.setFrameQueueCapacity(1);
        while (rgbImage == null) {
            try {
                closeableFrame = this.vuforia.getFrameQueue().take();
                long numImages = closeableFrame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (closeableFrame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgbImage = closeableFrame.getImage(i);
                        if (rgbImage != null) {
                            break;
                        }
                    }
                }
            } catch (InterruptedException exc) {

            } finally {
                if (closeableFrame != null) closeableFrame.close();
            }
        }

        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            String pathex = Environment.getExternalStorageDirectory().getAbsolutePath().toString();

            //String path = Environment.getExternalStoragePublicDirectory("Download").toString();
            FileOutputStream out = null;

            String bitmapName;
            String croppedBitmapName;

            if (red) {
                bitmapName = "BitmapRED.png";
                croppedBitmapName = "BitmapCroppedRED.png";
            } else {
                bitmapName = "BitmapBLUE.png";
                croppedBitmapName = "BitmapCroppedBLUE.png";
            }

            //Save bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, bitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }

            int cropStartX;
            int cropStartY;
            int cropWidth;
            int cropHeight;

            if (red) {
               // cropStartX = (int) ((120.0 / 480.0) * bitmap.getWidth());
                 cropStartX = (int) ((0.0/ 640.0) * bitmap.getWidth()); ;

//                cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                cropStartY = (int) ((250.0 / 480.0) * bitmap.getHeight());

                cropWidth = (int) ((640.0 / 640.0) * bitmap.getWidth());
                cropHeight = (int) ((63.0 / 480.0) * bitmap.getHeight());
            } else {
                cropStartX = (int) ((301.0 / 640.0) * bitmap.getWidth());
                cropStartY = (int) ((343.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((685.0 / 640.0) * bitmap.getWidth());
                cropHeight = (int) ((73.0 / 480.0) * bitmap.getHeight());
            }

         /*   if (red) {
                cropStartX = (int) ((120.0 / 480.0) * bitmap.getWidth());
                cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((590.0 / 480.0) * bitmap.getWidth());
                cropHeight = (int) ((170.0 / 480.0) * bitmap.getHeight());
            } else {
                cropStartX = (int) ((370.0 / 640.0) * bitmap.getWidth());
                cropStartY = (int) ((130.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((890.0 / 640.0) * bitmap.getWidth());
                cropHeight = (int) ((165.0 / 480.0) * bitmap.getHeight());
            }
*/


    /*         DbgLog.msg("10435 vuforiascan"
                    + " cropStartX: " + cropStartX
                    + " cropStartY: " + cropStartY
                    + " cropWidth: " + cropWidth
                    + " cropHeight: " + cropHeight
                    + " Width: " + bitmap.getWidth()
                    + " Height: " + bitmap.getHeight()
          );
 */

            bitmap = createBitmap(bitmap, cropStartX, cropStartY, cropWidth, cropHeight); //Cropped Bitmap to show only stones

            // Save cropped bitmap to file
            if (saveBitmaps) {
                try {
                    File file = new File(path, croppedBitmapName);
                    out = new FileOutputStream(file);
                    bitmap.compress(Bitmap.CompressFormat.PNG, 100, out);
                } catch (Exception e) {
                    e.printStackTrace();
                } finally {
                    try {
                        if (out != null) {
                            out.flush();
                            out.close();
                        }
                    } catch (IOException e) {
                        e.printStackTrace();
                    }
                }
            }
            bitmap =  createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time


            int height;
            int width;
            int pixel;
            int bitmapWidth = bitmap.getWidth();
            int bitmapHeight = bitmap.getHeight();
            int colWidth = (int) ((double) bitmapWidth / 6.0);
            int colorLStartCol = (int) ((double) bitmapWidth * (1.0 / 6.0) - ((double) colWidth / 1.1));
            int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
            int colorRStartCol = (int) ((double) bitmapWidth * (5.0 / 6.0) - ((double) colWidth / 9.0));

            for (height = 0; height < bitmapHeight; ++height) {
                for (width = colorLStartCol; width < colorLStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
                /*
                    if (Color.red(pixel) > redThreshold && Color.green(pixel) > greenThreshold && Color.blue(pixel) < blueThreshold) {
                        yellowCountL += Color.red(pixel);
                    }
                 */
                    if (Color.red(pixel) < redThreshold && Color.green(pixel) < greenThreshold && Color.blue(pixel) > blueThreshold) {
                        yellowCountL += Color.blue(pixel);
                    }

                }
                for (width = colorCStartCol; width < colorCStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
/*
                    if (Color.red(pixel) > redThreshold && Color.green(pixel) > greenThreshold && Color.blue(pixel) < blueThreshold) {
                        yellowCountC += Color.red(pixel);
                    }
  */                 if (Color.red(pixel) < redThreshold && Color.green(pixel) < greenThreshold && Color.blue(pixel) > blueThreshold) {
                        yellowCountC += Color.blue(pixel);
                    }


                }

                for (width = colorRStartCol; width < colorRStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);

              /*      if (Color.red(pixel) > redThreshold && Color.green(pixel) > greenThreshold && Color.blue(pixel) < blueThreshold) {
                        yellowCountR += Color.red(pixel);
                    }
                */
                    if (Color.red(pixel) < redThreshold && Color.green(pixel) < greenThreshold && Color.blue(pixel) > blueThreshold) {
                        yellowCountR += Color.blue(pixel);
                    }

                }
            }
        }

        double YellowCountFinalL = yellowCountL;
        double YellowCountFinalC = yellowCountC;
        double YellowCountFinalR = yellowCountR;


        capElementPos pos;
        capElementPositionData posData = new capElementPositionData();

        if (YellowCountFinalL > YellowCountFinalC && YellowCountFinalL > YellowCountFinalR) {
            pos = capElementPos.LEFT;
        } else if (YellowCountFinalC > YellowCountFinalL && YellowCountFinalC > YellowCountFinalR) {
            pos = capElementPos.CENTER;
        } else {
            pos = capElementPos.RIGHT;
        }
    posData.yellowCountCenter = YellowCountFinalC;
    posData.yellowCountLeft = YellowCountFinalL;
    posData.yellowCountRight = YellowCountFinalR;
    posData.capElementPosition = pos;
        return posData;

    }
}