package org.darbots.darbotsftclib.season_specific.skystone.tfod_detection;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.darbots.darbotsftclib.game_specific.AllianceType;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.RobotPoint2D;
import org.darbots.darbotsftclib.libcore.calculations.dimentional_calculation.XYPlaneCalculations;
import org.darbots.darbotsftclib.libcore.integratedfunctions.image_processing.FTCImageUtility;
import org.darbots.darbotsftclib.libcore.templates.other_sensors.RobotCamera;
import org.darbots.darbotsftclib.season_specific.skystone.SkyStonePosition;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;

public class SkyStoneStoneDifferentiation {
    public enum StoneType{
        STONE,
        SKYSTONE
    }

    public static class RecognitionResult{
        private StoneType m_StoneType;
        private float m_Left;
        private float m_Top;
        private float m_Width;
        private float m_Height;
        private float m_ImageWidth;
        private float m_ImageHeight;
        private float m_Confidence;

        public RecognitionResult(StoneType stoneType, float Left, float Top, float Width, float Height, float ImageWidth, float ImageHeight, float Confidence){
            this.m_StoneType = stoneType;
            this.m_Left = Left;
            this.m_Top = Top;
            this.m_Width = Width;
            this.m_Height = Height;
            this.m_ImageWidth = ImageWidth;
            this.m_ImageHeight = ImageHeight;
            this.m_Confidence = Confidence;
        }
        public RecognitionResult(RecognitionResult otherResult){
            this.m_StoneType = otherResult.m_StoneType;
            this.m_Left = otherResult.m_Left;
            this.m_Top = otherResult.m_Top;
            this.m_Width = otherResult.m_Width;
            this.m_Height = otherResult.m_Height;
            this.m_ImageWidth = otherResult.m_ImageWidth;
            this.m_ImageHeight = otherResult.m_ImageHeight;
            this.m_Confidence = otherResult.m_Confidence;
        }
        public RecognitionResult(Recognition TFODRecognition){
            this.m_StoneType = TFODRecognition.getLabel().equals("Stone") ? StoneType.STONE : StoneType.SKYSTONE;
            this.m_Left = TFODRecognition.getLeft();
            this.m_Top = TFODRecognition.getTop();
            this.m_Width = TFODRecognition.getWidth();
            this.m_Height = TFODRecognition.getHeight();
            this.m_ImageWidth = TFODRecognition.getImageWidth();
            this.m_ImageHeight = TFODRecognition.getImageHeight();
            this.m_Confidence = TFODRecognition.getConfidence();
        }

        public StoneType getStoneType(){
            return this.m_StoneType;
        }
        public void setStoneType(StoneType stoneType){
            this.m_StoneType = stoneType;
        }
        public float getLeft(){
            return this.m_Left;
        }
        public void setLeft(float left){
            this.m_Left = left;
        }
        public float getTop(){
            return this.m_Top;
        }
        public void setTop(float top){
            this.m_Top = top;
        }
        public float getWidth(){
            return this.m_Width;
        }

        public void setWidth(float width){
            this.m_Width = width;
        }

        public float getHeight(){
            return this.m_Height;
        }
        public void setHeight(float height){
            this.m_Height = height;
        }
        public float getRight(){
            return this.m_Left + this.m_Width;
        }
        public float getBottom(){
            return this.m_Top + this.m_Height;
        }
        public float getImageWidth(){
            return this.m_ImageWidth;
        }
        public void setImageWidth(float ImageWidth){
            this.m_ImageWidth = ImageWidth;
        }
        public float getImageHeight(){
            return this.m_ImageHeight;
        }
        public void setImageHeight(float ImageHeight){
            this.m_ImageWidth = ImageHeight;
        }
        public float getConfidence(){
            return this.m_Confidence;
        }
        public void setConfidence(float Confidence){
            this.m_Confidence = Confidence;
        }

    }
    private RobotCamera m_Camera;
    private TFObjectDetector m_TFOD;
    private HardwareMap m_HardwareMap;
    private boolean m_Preview;
    private boolean m_Activated = false;
    private double m_MinimumConfidence = 0.7;
    public SkyStoneStoneDifferentiation(RobotCamera Camera, HardwareMap HardwareList, boolean Preview, double MinimumConfidence) {
        this.m_Camera = Camera;
        this.m_HardwareMap = HardwareList;
        this.m_Preview = Preview;
        this.m_MinimumConfidence = MinimumConfidence;

        __initTfod();
    }

    public void terminate(){
        if(m_TFOD != null){
            m_TFOD.shutdown();
        }
    }

    public ArrayList<RecognitionResult> getUpdatedRecognitions(){
        if(m_TFOD == null){
            return null;
        }
        List<Recognition> updatedRecognitions = m_TFOD.getUpdatedRecognitions();
        if (updatedRecognitions != null) {
            ArrayList<RecognitionResult> ResultArray = new ArrayList<RecognitionResult>();
            // step through the list of recognitions and display boundary info.
            int i = 0;
            for (Recognition recognition : updatedRecognitions) {
                ResultArray.add(new RecognitionResult(recognition));
            }
            return ResultArray;
        }else{
            return null;
        }
    }

    public SkyStonePosition Sample_Standard_Three_Stone(AllianceType alliance, double cameraRotation){
        int result = 0; //0 = unknown, otherwise it is the # of stone from the left, starting from 1.
        ArrayList<RecognitionResult> recognitionResults = this.getUpdatedRecognitions();
        if(recognitionResults == null || recognitionResults.size() != 3){
            return SkyStonePosition.UNKNOWN;
        }

        RobotPoint2D imageMidPoint = new RobotPoint2D(
                recognitionResults.get(0).getImageWidth() / 2.0,
                recognitionResults.get(0).getImageHeight() / 2.0
        );

        ArrayList<RobotPoint2D> stonePositions = new ArrayList();
        RobotPoint2D skystonePosition = null;

        for(int i=0; i<recognitionResults.size();i++){
            RecognitionResult currentResult = recognitionResults.get(i);
            RobotPoint2D currentPoint = new RobotPoint2D(
                    (currentResult.getLeft() + currentResult.getRight()) / 2.0,
                    (currentResult.getTop() + currentResult.getBottom()) / 2.0
            );
            currentPoint = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(currentPoint,imageMidPoint,cameraRotation);
            if(currentResult.getStoneType() == StoneType.SKYSTONE){
                skystonePosition = currentPoint;
            }else{
                stonePositions.add(currentPoint);
            }
        }

        if(skystonePosition == null){
            return SkyStonePosition.UNKNOWN;
        }

        if((skystonePosition.X >= stonePositions.get(0).X && skystonePosition.X <= stonePositions.get(1).X) || (skystonePosition.X >= stonePositions.get(1).X && skystonePosition.X <= stonePositions.get(0).X)){
            result = 2;
        }else if(skystonePosition.X <= stonePositions.get(0).X && skystonePosition.X <= stonePositions.get(1).X){
            result = 1;
        }else { //skystonePosition.X >= stonePositions.get(0).X && skystonePosition.X >= stonePositions.get(1).X
            result = 3;
        }
        if(alliance == AllianceType.BLUE){
            switch(result){
                case 1:
                    return SkyStonePosition.NEXT_TO_BRIDGE;
                case 2:
                    return SkyStonePosition.MIDDLE;
                case 3:
                    return SkyStonePosition.NEXT_TO_WALL;
                default:
                    return SkyStonePosition.UNKNOWN;
            }
        }else { //alliance == AllianceType.RED
            switch(result){
                case 1:
                    return SkyStonePosition.NEXT_TO_WALL;
                case 2:
                    return SkyStonePosition.MIDDLE;
                case 3:
                    return SkyStonePosition.NEXT_TO_BRIDGE;
                default:
                    return SkyStonePosition.UNKNOWN;
            }
        }
    }

    public SkyStonePosition Sample_Middle_Line(AllianceType allianceType, double cameraRotation){
        int result = 0; //0 = unknown, otherwise it is the # of stone from the left, starting from 1.
        ArrayList<RecognitionResult> recognitionResults = this.getUpdatedRecognitions();
        if(recognitionResults == null || recognitionResults.size() != 3){
            return SkyStonePosition.UNKNOWN;
        }

        RobotPoint2D imageMidPoint = new RobotPoint2D(
                recognitionResults.get(0).getImageWidth() / 2.0,
                recognitionResults.get(0).getImageHeight() / 2.0
        );

        RobotPoint2D skystoneStartPosition = null;
        RobotPoint2D skystoneEndPosition = null;

        for(int i=0; i<recognitionResults.size();i++){
            RecognitionResult currentResult = recognitionResults.get(i);
            if(currentResult.getStoneType() == StoneType.SKYSTONE) {
                RobotPoint2D currentStartPoint = new RobotPoint2D(
                        currentResult.getLeft(),
                        currentResult.getTop()
                );
                RobotPoint2D currentEndPoint = new RobotPoint2D(
                        currentResult.getRight(),
                        currentResult.getBottom()
                );
                currentStartPoint = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(currentStartPoint,imageMidPoint,cameraRotation);
                currentEndPoint = XYPlaneCalculations.rotatePointAroundFixedPoint_Deg(currentEndPoint,imageMidPoint,cameraRotation);

                //because the minX and minY are not guaranteed to be minX and minY anymore after rotation, here we are getting new start and end points, the bounding box does not change though.
                RobotPoint2D[] actualStartAndEnd = FTCImageUtility.getStartPointAndEndPoint(currentStartPoint,currentEndPoint);

                skystoneStartPosition = actualStartAndEnd[0];
                skystoneEndPosition = actualStartAndEnd[1];
                break;
            }
        }

        if(skystoneStartPosition == null || skystoneEndPosition == null){
            return SkyStonePosition.UNKNOWN;
        }

        if(skystoneStartPosition.X <= imageMidPoint.X && skystoneEndPosition.X >= imageMidPoint.X){
            result = 2;
        }else if(skystoneStartPosition.X <= imageMidPoint.X && skystoneEndPosition.X <= imageMidPoint.X){
            result = 1;
        }else { //skystonePosition.X >= imageMidPoint.X && skystonePosition.X >= imageMidPoint.X
            result = 3;
        }
        if(allianceType == AllianceType.BLUE){
            switch(result){
                case 1:
                    return SkyStonePosition.NEXT_TO_BRIDGE;
                case 2:
                    return SkyStonePosition.MIDDLE;
                case 3:
                    return SkyStonePosition.NEXT_TO_WALL;
                default:
                    return SkyStonePosition.UNKNOWN;
            }
        }else { //!blueSide, redSide
            switch(result){
                case 1:
                    return SkyStonePosition.NEXT_TO_WALL;
                case 2:
                    return SkyStonePosition.MIDDLE;
                case 3:
                    return SkyStonePosition.NEXT_TO_BRIDGE;
                default:
                    return SkyStonePosition.UNKNOWN;
            }
        }
    }

    public double getMinimumConfidence(){
        return this.m_MinimumConfidence;
    }


    public RobotCamera getCamera(){
        return this.m_Camera;
    }
    public boolean isPreview(){
        return this.m_Preview;
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void __initTfod() {

        int tfodMonitorViewId = 0;
        TFObjectDetector.Parameters tfodParameters = null;
        if(m_Preview) {
            tfodMonitorViewId = m_HardwareMap.appContext.getResources().getIdentifier(
                    "tfodMonitorViewId", "id", m_HardwareMap.appContext.getPackageName());
            tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        }else{
            tfodParameters = new TFObjectDetector.Parameters();
        }
        tfodParameters.minimumConfidence = this.m_MinimumConfidence;
        m_TFOD = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, this.m_Camera.getVuforia());
        m_TFOD.loadModelFromAsset("Skystone.tflite", "Stone", "Skystone");
    }

    public void setActivated(boolean enabled){
        if(m_TFOD == null){
            return;
        }
        if(enabled){
            this.m_TFOD.activate();
            this.m_Activated = true;
        }else{
            this.m_TFOD.deactivate();
            this.m_Activated = false;
        }
    }

    public boolean isActivated(){
        if(this.m_TFOD == null){
            return false;
        }else{
            return this.m_Activated;
        }
    }

}
