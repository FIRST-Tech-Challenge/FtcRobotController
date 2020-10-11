package org.darbots.darbotsftclib.season_specific.skystone.darbots_pixel_skystone_detection;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.darbots.darbotsftclib.libcore.integratedfunctions.image_processing.FTCImageUtility;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStoneType;

public class DarbotsPixelSkyStoneSampler {
    public static final float BRIGHTNESS_SKYSTONE = 0.35f;
    public static final float HUE_MIN_STONE = 45.0f;
    public static final float HUE_MAX_STONE = 75.0f;

    private RobotCamera m_Camera;
    public DarbotsPixelSkyStoneSampler(RobotCamera camera){
        this.m_Camera = camera;
    }
    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public SkyStoneType getStoneType(int originalImageWidth, int originalImageHeight, int sampleImageWidth, int sampleImageHeight, int originalImageStartX, int originalImageStartY, int originalImageEndX, int originalImageEndY){
        Bitmap frame = this.getFrame();
        if(frame == null){
            return SkyStoneType.UNKNOWN;
        }
        int countedColor = FTCImageUtility.countShrinkedScaledAverageColor(frame,sampleImageWidth,sampleImageHeight,originalImageWidth,originalImageHeight,originalImageStartX,originalImageStartY,originalImageEndX,originalImageEndY);
        float[] countedHSV = new float[3];
        Color.colorToHSV(countedColor,countedHSV);

        float hue = countedHSV[0];
        float saturation = countedHSV[1];
        float brightness = countedHSV[2];
        if(brightness <= BRIGHTNESS_SKYSTONE){
            return SkyStoneType.SKYSTONE;
        }
        if(hue >= HUE_MIN_STONE && hue <= HUE_MAX_STONE){
            return SkyStoneType.STONE;
        }
        return SkyStoneType.UNKNOWN;
    }
    public SkyStonePosition sample(int originalImageWidth, int originalImageHeight, int sampleImageWidth, int sampleImageHeight, int WallStoneStartX, int WallStoneStartY, int WallStoneEndX, int WallStoneEndY, int CenterStoneStartX, int CenterStoneStartY, int CenterStoneEndX, int CenterStoneEndY, int BridgeStoneStartX, int BridgeStoneStartY, int BridgeStoneEndX, int BridgeStoneEndY){
        Bitmap frame = this.getFrame();
        if(frame == null){
            return SkyStonePosition.UNKNOWN;
        }
        return this.sample(frame,false, originalImageWidth,originalImageHeight,sampleImageWidth,sampleImageHeight,WallStoneStartX,WallStoneStartY,WallStoneEndX,WallStoneEndY,CenterStoneStartX,CenterStoneStartY,CenterStoneEndX,CenterStoneEndY,BridgeStoneStartX,BridgeStoneStartY,BridgeStoneEndX,BridgeStoneEndY);
    }
    public Bitmap getFrame(){
        int trial = 0;
        Bitmap frame = null;
        while(frame == null){
            trial++;
            if(trial > 3){
                return null;
            }
            frame = this.m_Camera.getFrame();
        }
        return frame;
    }
    public SkyStonePosition sample(Bitmap frame, boolean drawWhereItIs ,int originalImageWidth, int originalImageHeight, int sampleImageWidth, int sampleImageHeight, int WallStoneStartX, int WallStoneStartY, int WallStoneEndX, int WallStoneEndY, int CenterStoneStartX, int CenterStoneStartY, int CenterStoneEndX, int CenterStoneEndY, int BridgeStoneStartX, int BridgeStoneStartY, int BridgeStoneEndX, int BridgeStoneEndY){
        Bitmap scaledBitmap = FTCImageUtility.getScaledImage(frame,sampleImageWidth,sampleImageHeight);

        if(drawWhereItIs) {
            this.drawSamplingBoxOnFrame(frame,WallStoneStartX,WallStoneStartY,WallStoneEndX,WallStoneEndY,CenterStoneStartX,CenterStoneStartY,CenterStoneEndX,CenterStoneEndY,BridgeStoneStartX,BridgeStoneStartY,BridgeStoneEndX,BridgeStoneEndY);
        }

        int WallStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,WallStoneStartX,WallStoneStartY,WallStoneEndX,WallStoneEndY);
        float[] WallStoneHSV = new float[3];
        Color.colorToHSV(WallStoneColor,WallStoneHSV);
        float WallStoneBrightness = WallStoneHSV[2];

        int CenterStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,CenterStoneStartX,CenterStoneStartY,CenterStoneEndX,CenterStoneEndY);
        float[] CenterStoneHSV = new float[3];
        Color.colorToHSV(CenterStoneColor,CenterStoneHSV);
        float CenterStoneBrightness = CenterStoneHSV[2];

        int BridgeStoneColor = FTCImageUtility.countShrinkedScaledAverageColor(scaledBitmap,originalImageWidth,originalImageHeight,BridgeStoneStartX,BridgeStoneStartY,BridgeStoneEndX,BridgeStoneEndY);
        float[] BridgeStoneHSV = new float[3];
        Color.colorToHSV(BridgeStoneColor,BridgeStoneHSV);
        float BridgeStoneBrightness = BridgeStoneHSV[2];

        if(WallStoneBrightness <= CenterStoneBrightness && WallStoneBrightness <= BridgeStoneBrightness){
            return SkyStonePosition.NEXT_TO_WALL;
        }else if(CenterStoneBrightness <= WallStoneBrightness && CenterStoneBrightness <= BridgeStoneBrightness){
            return SkyStonePosition.MIDDLE;
        }else{
            return SkyStonePosition.NEXT_TO_BRIDGE;
        }
    }
    public void drawSamplingBoxOnFrame(Bitmap frame, int WallStoneStartX, int WallStoneStartY, int WallStoneEndX, int WallStoneEndY, int CenterStoneStartX, int CenterStoneStartY, int CenterStoneEndX, int CenterStoneEndY, int BridgeStoneStartX, int BridgeStoneStartY, int BridgeStoneEndX, int BridgeStoneEndY){
        Paint samplerPainter = new Paint();
        samplerPainter.setColor(Color.BLACK);

        samplerPainter.setTextSize(32);
        samplerPainter.setStrokeWidth(3.0f);
        samplerPainter.setTextAlign(Paint.Align.LEFT);
        Canvas bitmapCanvas = new Canvas(frame);
        {
            drawRect(bitmapCanvas,samplerPainter,WallStoneStartX, WallStoneStartY, WallStoneEndX, WallStoneEndY);
            bitmapCanvas.drawText("Wall Stone", WallStoneStartX, WallStoneStartY, samplerPainter);
        }
        {
            drawRect(bitmapCanvas,samplerPainter,CenterStoneStartX, CenterStoneStartY, CenterStoneEndX, CenterStoneEndY);
            bitmapCanvas.drawText("Center Stone", CenterStoneStartX, CenterStoneStartY, samplerPainter);
        }
        {
            drawRect(bitmapCanvas,samplerPainter,BridgeStoneStartX,BridgeStoneStartY,BridgeStoneEndX,BridgeStoneEndY);
            bitmapCanvas.drawText("Bridge Stone", BridgeStoneStartX, BridgeStoneStartY, samplerPainter);
        }
    }
    public static void drawRect(Canvas canvas, Paint paint, float startX, float startY, float endX, float endY){
        canvas.drawLine(startX,startY,endX,startY,paint);
        canvas.drawLine(endX,startY,endX,endY,paint);
        canvas.drawLine(endX,endY,startX,endY,paint);
        canvas.drawLine(startX,endY,startX,startY,paint);
    }
}
