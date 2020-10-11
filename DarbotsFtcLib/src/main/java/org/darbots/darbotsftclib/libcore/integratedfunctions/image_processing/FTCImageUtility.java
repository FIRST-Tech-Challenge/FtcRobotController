package org.darbots.darbotsftclib.libcore.integratedfunctions.image_processing;

import android.graphics.Bitmap;
import android.graphics.Color;
import androidx.annotation.ColorInt;

import org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation.AlgebraicCalculations;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;

public class FTCImageUtility {
    @ColorInt
    public static int countScaledAverageColor(Bitmap image, int originalWidth, int originalHeight, int originalStartX, int originalStartY, int originalEndX, int originalEndY){
        //rescale the bitmap
        Bitmap scaledBitmap = Bitmap.createScaledBitmap(image,originalWidth,originalHeight,true);
        return countAverageColor(scaledBitmap,originalStartX,originalEndY,originalEndX,originalEndY);
    }
    @ColorInt
    public static int countAverageColor(Bitmap image, int startX, int startY, int endX, int endY){
        //Crop Bitmap
        int imageWidth = endX - startX + 1;
        int imageHeight = endY - endY + 1;
        Bitmap croppedIMG = Bitmap.createBitmap(image,startX,startY,imageWidth,imageHeight);
        long RedChannelCounter = 0L, GreenChannelCounter = 0L, BlueChannelCounter = 0L, AlphaChannelCounter = 0L;
        int pixelNum = imageWidth * imageHeight;
        for(int x = 0; x < imageWidth; x++){
            for(int y = 0; y < imageHeight; y++){
                int pixelColor = croppedIMG.getPixel(x,y);
                int pixelR, pixelG, pixelB, pixelA;
                pixelR = Color.red(pixelColor);
                pixelG = Color.green(pixelColor);
                pixelB = Color.blue(pixelColor);
                pixelA = Color.alpha(pixelColor);
                RedChannelCounter += pixelR;
                GreenChannelCounter += pixelG;
                BlueChannelCounter += pixelB;
                AlphaChannelCounter += pixelA;
            }
        }
        int avgRed = (int) Math.round(((double) RedChannelCounter) / pixelNum);
        int avgGreen = (int) Math.round(((double) GreenChannelCounter) / pixelNum);
        int avgBlue = (int) Math.round(((double) BlueChannelCounter) / pixelNum);
        int avgAlpha = (int) Math.round(((double) AlphaChannelCounter) / pixelNum);
        int colorVal = Color.argb(avgAlpha,avgRed,avgGreen,avgBlue);
        return colorVal;
    }

    public static Bitmap getScaledImage(Bitmap image, int newWidth, int newHeight){
        return Bitmap.createScaledBitmap(image,newWidth,newHeight,true);
    }

    @ColorInt
    public static int countShrinkedScaledAverageColor(Bitmap image, int sampleWidth, int sampleHeight, int originalWidth, int originalHeight, int originalStartX, int originalStartY, int originalEndX, int originalEndY){
        //rescale the bitmap
        Bitmap scaledBitmap = getScaledImage(image,sampleWidth,sampleHeight);
        int sampleStartX = AlgebraicCalculations.map(originalStartX,0,originalWidth,0,sampleWidth);
        int sampleEndX = AlgebraicCalculations.map(originalEndX,0,originalWidth,0,sampleWidth);
        int sampleStartY = AlgebraicCalculations.map(originalStartY,0,originalHeight,0,sampleHeight);
        int sampleEndY = AlgebraicCalculations.map(originalEndY,0,originalHeight,0,sampleHeight);
        return countAverageColor(scaledBitmap,sampleStartX,sampleStartY,sampleEndX,sampleEndY);
    }

    @ColorInt
    public static int countShrinkedScaledAverageColor(Bitmap scaledImage, int originalWidth, int originalHeight, int originalStartX, int originalStartY, int originalEndX, int originalEndY){
        int sampleWidth = scaledImage.getWidth();
        int sampleHeight = scaledImage.getHeight();
        int sampleStartX = AlgebraicCalculations.map(originalStartX,0,originalWidth,0,sampleWidth);
        int sampleEndX = AlgebraicCalculations.map(originalEndX,0,originalWidth,0,sampleWidth);
        int sampleStartY = AlgebraicCalculations.map(originalStartY,0,originalHeight,0,sampleHeight);
        int sampleEndY = AlgebraicCalculations.map(originalEndY,0,originalHeight,0,sampleHeight);
        return countAverageColor(scaledImage,sampleStartX,sampleStartY,sampleEndX,sampleEndY);
    }

    public static RobotPoint2D[] getStartPointAndEndPoint(RobotPoint2D point1, RobotPoint2D point2){
        double minX = Math.min(point1.X, point2.X), maxX = Math.max(point1.X, point2.X);
        double minY = Math.min(point1.Y, point2.Y), maxY = Math.max(point2.Y, point2.Y);
        RobotPoint2D[] returnVal = new RobotPoint2D[2];
        returnVal[0] = new RobotPoint2D(minX,minY);
        returnVal[1] = new RobotPoint2D(maxX,maxY);
        return returnVal;
    }
}
