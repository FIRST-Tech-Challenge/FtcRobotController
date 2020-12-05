package org.firstinspires.ftc.teamcode.team10515.auto;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.os.Environment;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.team10515.DbgLog;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;

import static android.graphics.Bitmap.createBitmap;
import static android.graphics.Bitmap.createScaledBitmap;

public class UGVuforia {

    VuforiaLocalizer vuforia;

    public UGVuforia(VuforiaLocalizer vuforia) {
        this.vuforia = vuforia;
    }

    public enum skystonePos {
        LEFT, CENTER, RIGHT
    }

    public skystonePos GetPosition(boolean saveBitmaps, boolean red) {
        Image rgbImage = null;

        double yellowCountL = 1, yellowCountC = 1, yellowCountR = 1;
        double blackCountL = 1, blackCountC = 1, blackCountR = 1;

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
                DbgLog.msg("About to close the frame  scanning ");
                if (closeableFrame != null) closeableFrame.close();
                DbgLog.msg("Frame is closed ");
            }
        }



        if (rgbImage != null) {

            // copy the bitmap from the Vuforia frame
            Bitmap bitmap = Bitmap.createBitmap(rgbImage.getWidth(), rgbImage.getHeight(), Bitmap.Config.RGB_565);
            bitmap.copyPixelsFromBuffer(rgbImage.getPixels());

            String path = Environment.getExternalStorageDirectory().toString();
            FileOutputStream out = null;

            String bitmapName;
            String croppedBitmapName;

            if (red) {
                bitmapName = "imageRED.png";
                croppedBitmapName = "croppedRED.png";
            } else {
                bitmapName = "imageBLUE.png";
                croppedBitmapName = "croppedBLUE.png";
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

            int cropStartX, cropStartY, cropWidth, cropHeight;

            if (red) {
                //cropStartX = (int) ((120.0 / 720.0) * bitmap.getWidth());
                //cropStartY = (int) ((100.0 / 480.0) * bitmap.getHeight());
                //cropWidth = (int) ((590.0 / 720.0) * bitmap.getWidth());
                //cropHeight = (int) ((103.0 / 480.0) * bitmap.getHeight());

                cropStartX = (int) ((20.0 / 720.0) * bitmap.getWidth());
                cropStartY = (int) ((370.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((550.0 / 720.0) * bitmap.getWidth());
                cropHeight = (int) ((90.0 / 480.0) * bitmap.getHeight());
            } else {
                //cropStartX = (int) ((370.0 / 1280.0) * bitmap.getWidth());
                //cropStartY = (int) ((130.0 / 720.0) * bitmap.getHeight());
                //cropWidth = (int) ((890.0 / 1280.0) * bitmap.getWidth());
                //cropHeight = (int) ((165.0 / 720.0) * bitmap.getHeight());
                cropStartX = (int) ((20.0 / 720.0) * bitmap.getWidth());
                cropStartY = (int) ((350.0 / 480.0) * bitmap.getHeight());
                cropWidth = (int) ((560.0 / 720.0) * bitmap.getWidth());
                cropHeight = (int) ((90.0 / 480.0) * bitmap.getHeight());
            }

            DbgLog.msg("XV scanning "
                    + " cropStartX: " + cropStartX
                    + " cropStartY: " + cropStartY
                    + " cropWidth: " + cropWidth
                    + " cropHeight: " + cropHeight
                    + " Width: " + bitmap.getWidth()
                    + " Height: " + bitmap.getHeight()
            );

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
            bitmap = createScaledBitmap(bitmap, 110, 20, true); //Compress bitmap to reduce scan time

            int height;
            int width;
            int pixel;
            int bitmapWidth = bitmap.getWidth();
            int bitmapHeight = bitmap.getHeight();
            int colWidth = (int) ((double) bitmapWidth / 6.0);
            int colorLStartCol = (int) ((double) bitmapWidth * (1.0 / 6.0) - ((double) colWidth / 2.0));
            int colorCStartCol = (int) ((double) bitmapWidth * (3.0 / 6.0) - ((double) colWidth / 2.0));
            int colorRStartCol = (int) ((double) bitmapWidth * (5.0 / 6.0) - ((double) colWidth / 2.0));

            for (height = 0; height < bitmapHeight; ++height) {
                for (width = colorLStartCol; width < colorLStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);
                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountL += Color.red(pixel);
                        blackCountL += Color.blue(pixel);
                    }

                    //colorcountL += Color.red(pixel) + Color.green(pixel) + Color.blue(pixel);
                }
                for (width = colorCStartCol; width < colorCStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);

                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountC += Color.red(pixel);
                        blackCountC += Color.blue(pixel);
                    }
                }

                for (width = colorRStartCol; width < colorRStartCol + colWidth; ++width) {
                    pixel = bitmap.getPixel(width, height);

                    if (Color.red(pixel) < 200 || Color.green(pixel) < 200 || Color.blue(pixel) < 200) {
                        yellowCountR += Color.red(pixel);
                        blackCountR += Color.blue(pixel);
                    }
                }
            }
        }

        double blackYellowRatioL = blackCountL / yellowCountL;
        double blackYellowRatioC = blackCountC / yellowCountC;
        double blackYellowRatioR = blackCountR / yellowCountR;

        skystonePos pos;

        if (blackYellowRatioL > blackYellowRatioC && blackYellowRatioL > blackYellowRatioR) {
            pos = skystonePos.LEFT;
        } else if (blackYellowRatioC > blackYellowRatioL && blackYellowRatioC > blackYellowRatioR) {
            pos = skystonePos.CENTER;
        } else {
            pos = skystonePos.RIGHT;
        }

        DbgLog.msg("Black/Yellow Left: " + blackCountL + "/" + yellowCountL);
        DbgLog.msg("Black/Yellow Center: " + blackCountC + "/" + yellowCountC);
        DbgLog.msg("Black/Yellow Right: " + blackCountR + "/" + yellowCountR);

        return pos;
    }
}