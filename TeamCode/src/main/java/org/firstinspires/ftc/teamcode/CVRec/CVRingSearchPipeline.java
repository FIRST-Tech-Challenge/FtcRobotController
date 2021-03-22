package org.firstinspires.ftc.teamcode.CVRec;

import android.util.Log;

import org.firstinspires.ftc.teamcode.skills.Geometry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

public class CVRingSearchPipeline extends CVPipelineBase {

    private ArrayList<CVRoi> regions = new ArrayList<>();
    private Mat YCrCb = new Mat();
    private Mat Cb = new Mat();
    private int numRegions;

    static final int REGION_WIDTH = 40;
    static final int REGION_HEIGHT = 60;

    static final int SINGLE_MAX = 118;

    private static String TAG = "CVRingSearchPipeline";


    public CVRingSearchPipeline(int resX, int resY){
        super(resX, resY);
        this.numRegions = getResolutionX()-REGION_WIDTH * getResolutionY()/REGION_HEIGHT;
    }

    private void buildRegions(){
        int numCols = getResolutionX()/REGION_WIDTH;
        Point intake = new Point(getResolutionX()*3/4, getResolutionY());
        Point robotCenter = new Point(getResolutionX()*3/4, getResolutionY() + ROBOT_CENTER * PIXELS_PER_INCH);
        int startX = 0;
        int startY = 0;
        Point pointA = new Point(startX, startY);
        Point pointB = new Point(REGION_WIDTH, REGION_HEIGHT);
        int colIndex = 0;
        int rowIndex = 1;
        for(int i = 0; i < numRegions; i++){
            Rect r = new Rect(pointA, pointB);
            Log.d(TAG, String.format("Rectangle %d. A: %.2f x %.2f  B:  %.2f x %.2f    ", i, pointA.x, pointA.y, pointB.x, pointB.y));
            Point regionCenter = new Point(pointA.x + REGION_WIDTH/2, pointA.y + REGION_HEIGHT/2);
            double distancePixels = Geometry.getDistance(intake.x, intake.y, regionCenter.x, regionCenter.y);
            double distanceInches = distancePixels / PIXELS_PER_INCH;
            double catet = regionCenter.x - intake.x;
            boolean clockwise = catet > 0;
            double distanceRobotCenterPixels = Geometry.getDistance(robotCenter.x, robotCenter.y, regionCenter.x, regionCenter.y);
            double angleDegrees = Math.toDegrees(Math.asin(Math.abs(catet)/distanceRobotCenterPixels));
            CVRoi roi = new CVRoi();
            roi.setIndex(i);
            roi.setAngle(angleDegrees);
            roi.setDistance(distanceInches);
            roi.setInput(Cb.submat(r));
            roi.setClockwise(clockwise);
            regions.add(roi);

            colIndex++;
            if (colIndex >= numCols){
                colIndex = 0;
                rowIndex++;
            }
            //next region
            pointA = new Point(startX + REGION_WIDTH*colIndex, startY + REGION_HEIGHT*(rowIndex - 1));
            pointB = new Point(startX + REGION_WIDTH*(colIndex+1), startY + REGION_HEIGHT*(rowIndex));
        }
    }

    @Override
    public void init(Mat input) {
        try {
            inputToCb(input);
            buildRegions();
        }
        catch (Exception ex){
            Log.e(TAG, "Error initializing the pipeline", ex);
        }
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        for(int i = 0; i < regions.size(); i++) {
            CVRoi roi = this.regions.get(i);
            double mean = (int) Core.mean(roi.getInput()).val[0];

           if(mean < SINGLE_MAX && !getTargets().contains(roi)) {
               roi.setMeanVal(mean);
               getTargets().add(roi);
           }
        }


        List<CVRoi> list = getTargets();
        Collections.sort(list, new Comparator<CVRoi>() {
            @Override
            public int compare(CVRoi cvRoi, CVRoi next) {
                return Double.compare(cvRoi.getMeanVal(), next.getMeanVal());
            }
        });

        if (list.size() > 0){
            setNearestTarget(list.get(0));
        }
        else{
            setNearestTarget(null);
        }

        if (list.size() > 1){
            setSecondTarget(list.get(1));
        }
        else{
            setSecondTarget(null);
        }

        if (neighbors(getNearestTarget(), getSecondTarget())){
            //merge if mean values are indicative of partial match.
            CVRoi merged  = new CVRoi();
            merged.setMerged(true);
            merged.setMeanVal((getNearestTarget().getMeanVal() + getSecondTarget().getMeanVal())/2);
            merged.setDistance((getNearestTarget().getDistance() + getSecondTarget().getDistance())/2);
            merged.setIndex(getNearestTarget().getIndex());
//            merged.setInput(getNearestTarget().getInput());
            double firstAngle = getNearestTarget().getAngle();
            if (!getNearestTarget().isClockwise()){
                firstAngle = -firstAngle;
            }
            double secondAngle = getSecondTarget().getAngle();
            if (!getSecondTarget().isClockwise()){
                secondAngle = -secondAngle;
            }
            double mergedAngle = (firstAngle + secondAngle)/2;
            merged.setClockwise(mergedAngle > 0);
            merged.setAngle(Math.abs(mergedAngle));
            nearestTarget = merged;
            secondTarget = null;
        }


        clearTargets();
        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        return input;
    }

    private void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    private boolean neighbors(CVRoi first, CVRoi next){
        if (first == null || next == null){
            return false;
        }

        int numCols = getResolutionX()/REGION_WIDTH;

        int firstIndex = first.getIndex();
        int nextIndex = next.getIndex();
        if ((firstIndex + 1 == nextIndex || firstIndex - 1 == nextIndex) ||
                (firstIndex - numCols + 1 == nextIndex || firstIndex - numCols - 1 == nextIndex) ||
                (firstIndex + numCols + 1 == nextIndex || firstIndex + numCols - 1 == nextIndex)
        ) {
            return true;
        }
        return false;
    }
}
