//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.DetectorParameters;

// C++: class DetectorParameters
//javadoc: DetectorParameters

public class DetectorParameters {

    protected final long nativeObj;
    protected DetectorParameters(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    // internal usage only
    public static DetectorParameters __fromPtr__(long addr) { return new DetectorParameters(addr); }

    //
    // C++: static Ptr_DetectorParameters create()
    //

    //javadoc: DetectorParameters::create()
    public static DetectorParameters create()
    {
        
        DetectorParameters retVal = DetectorParameters.__fromPtr__(create_0());
        
        return retVal;
    }


    //
    // C++: int DetectorParameters::adaptiveThreshWinSizeMin
    //

    //javadoc: DetectorParameters::get_adaptiveThreshWinSizeMin()
    public  int get_adaptiveThreshWinSizeMin()
    {
        
        int retVal = get_adaptiveThreshWinSizeMin_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::adaptiveThreshWinSizeMin
    //

    //javadoc: DetectorParameters::set_adaptiveThreshWinSizeMin(adaptiveThreshWinSizeMin)
    public  void set_adaptiveThreshWinSizeMin(int adaptiveThreshWinSizeMin)
    {
        
        set_adaptiveThreshWinSizeMin_0(nativeObj, adaptiveThreshWinSizeMin);
        
        return;
    }


    //
    // C++: int DetectorParameters::adaptiveThreshWinSizeMax
    //

    //javadoc: DetectorParameters::get_adaptiveThreshWinSizeMax()
    public  int get_adaptiveThreshWinSizeMax()
    {
        
        int retVal = get_adaptiveThreshWinSizeMax_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::adaptiveThreshWinSizeMax
    //

    //javadoc: DetectorParameters::set_adaptiveThreshWinSizeMax(adaptiveThreshWinSizeMax)
    public  void set_adaptiveThreshWinSizeMax(int adaptiveThreshWinSizeMax)
    {
        
        set_adaptiveThreshWinSizeMax_0(nativeObj, adaptiveThreshWinSizeMax);
        
        return;
    }


    //
    // C++: int DetectorParameters::adaptiveThreshWinSizeStep
    //

    //javadoc: DetectorParameters::get_adaptiveThreshWinSizeStep()
    public  int get_adaptiveThreshWinSizeStep()
    {
        
        int retVal = get_adaptiveThreshWinSizeStep_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::adaptiveThreshWinSizeStep
    //

    //javadoc: DetectorParameters::set_adaptiveThreshWinSizeStep(adaptiveThreshWinSizeStep)
    public  void set_adaptiveThreshWinSizeStep(int adaptiveThreshWinSizeStep)
    {
        
        set_adaptiveThreshWinSizeStep_0(nativeObj, adaptiveThreshWinSizeStep);
        
        return;
    }


    //
    // C++: double DetectorParameters::adaptiveThreshConstant
    //

    //javadoc: DetectorParameters::get_adaptiveThreshConstant()
    public  double get_adaptiveThreshConstant()
    {
        
        double retVal = get_adaptiveThreshConstant_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::adaptiveThreshConstant
    //

    //javadoc: DetectorParameters::set_adaptiveThreshConstant(adaptiveThreshConstant)
    public  void set_adaptiveThreshConstant(double adaptiveThreshConstant)
    {
        
        set_adaptiveThreshConstant_0(nativeObj, adaptiveThreshConstant);
        
        return;
    }


    //
    // C++: double DetectorParameters::minMarkerPerimeterRate
    //

    //javadoc: DetectorParameters::get_minMarkerPerimeterRate()
    public  double get_minMarkerPerimeterRate()
    {
        
        double retVal = get_minMarkerPerimeterRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::minMarkerPerimeterRate
    //

    //javadoc: DetectorParameters::set_minMarkerPerimeterRate(minMarkerPerimeterRate)
    public  void set_minMarkerPerimeterRate(double minMarkerPerimeterRate)
    {
        
        set_minMarkerPerimeterRate_0(nativeObj, minMarkerPerimeterRate);
        
        return;
    }


    //
    // C++: double DetectorParameters::maxMarkerPerimeterRate
    //

    //javadoc: DetectorParameters::get_maxMarkerPerimeterRate()
    public  double get_maxMarkerPerimeterRate()
    {
        
        double retVal = get_maxMarkerPerimeterRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::maxMarkerPerimeterRate
    //

    //javadoc: DetectorParameters::set_maxMarkerPerimeterRate(maxMarkerPerimeterRate)
    public  void set_maxMarkerPerimeterRate(double maxMarkerPerimeterRate)
    {
        
        set_maxMarkerPerimeterRate_0(nativeObj, maxMarkerPerimeterRate);
        
        return;
    }


    //
    // C++: double DetectorParameters::polygonalApproxAccuracyRate
    //

    //javadoc: DetectorParameters::get_polygonalApproxAccuracyRate()
    public  double get_polygonalApproxAccuracyRate()
    {
        
        double retVal = get_polygonalApproxAccuracyRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::polygonalApproxAccuracyRate
    //

    //javadoc: DetectorParameters::set_polygonalApproxAccuracyRate(polygonalApproxAccuracyRate)
    public  void set_polygonalApproxAccuracyRate(double polygonalApproxAccuracyRate)
    {
        
        set_polygonalApproxAccuracyRate_0(nativeObj, polygonalApproxAccuracyRate);
        
        return;
    }


    //
    // C++: double DetectorParameters::minCornerDistanceRate
    //

    //javadoc: DetectorParameters::get_minCornerDistanceRate()
    public  double get_minCornerDistanceRate()
    {
        
        double retVal = get_minCornerDistanceRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::minCornerDistanceRate
    //

    //javadoc: DetectorParameters::set_minCornerDistanceRate(minCornerDistanceRate)
    public  void set_minCornerDistanceRate(double minCornerDistanceRate)
    {
        
        set_minCornerDistanceRate_0(nativeObj, minCornerDistanceRate);
        
        return;
    }


    //
    // C++: int DetectorParameters::minDistanceToBorder
    //

    //javadoc: DetectorParameters::get_minDistanceToBorder()
    public  int get_minDistanceToBorder()
    {
        
        int retVal = get_minDistanceToBorder_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::minDistanceToBorder
    //

    //javadoc: DetectorParameters::set_minDistanceToBorder(minDistanceToBorder)
    public  void set_minDistanceToBorder(int minDistanceToBorder)
    {
        
        set_minDistanceToBorder_0(nativeObj, minDistanceToBorder);
        
        return;
    }


    //
    // C++: double DetectorParameters::minMarkerDistanceRate
    //

    //javadoc: DetectorParameters::get_minMarkerDistanceRate()
    public  double get_minMarkerDistanceRate()
    {
        
        double retVal = get_minMarkerDistanceRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::minMarkerDistanceRate
    //

    //javadoc: DetectorParameters::set_minMarkerDistanceRate(minMarkerDistanceRate)
    public  void set_minMarkerDistanceRate(double minMarkerDistanceRate)
    {
        
        set_minMarkerDistanceRate_0(nativeObj, minMarkerDistanceRate);
        
        return;
    }


    //
    // C++: int DetectorParameters::cornerRefinementMethod
    //

    //javadoc: DetectorParameters::get_cornerRefinementMethod()
    public  int get_cornerRefinementMethod()
    {
        
        int retVal = get_cornerRefinementMethod_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::cornerRefinementMethod
    //

    //javadoc: DetectorParameters::set_cornerRefinementMethod(cornerRefinementMethod)
    public  void set_cornerRefinementMethod(int cornerRefinementMethod)
    {
        
        set_cornerRefinementMethod_0(nativeObj, cornerRefinementMethod);
        
        return;
    }


    //
    // C++: int DetectorParameters::cornerRefinementWinSize
    //

    //javadoc: DetectorParameters::get_cornerRefinementWinSize()
    public  int get_cornerRefinementWinSize()
    {
        
        int retVal = get_cornerRefinementWinSize_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::cornerRefinementWinSize
    //

    //javadoc: DetectorParameters::set_cornerRefinementWinSize(cornerRefinementWinSize)
    public  void set_cornerRefinementWinSize(int cornerRefinementWinSize)
    {
        
        set_cornerRefinementWinSize_0(nativeObj, cornerRefinementWinSize);
        
        return;
    }


    //
    // C++: int DetectorParameters::cornerRefinementMaxIterations
    //

    //javadoc: DetectorParameters::get_cornerRefinementMaxIterations()
    public  int get_cornerRefinementMaxIterations()
    {
        
        int retVal = get_cornerRefinementMaxIterations_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::cornerRefinementMaxIterations
    //

    //javadoc: DetectorParameters::set_cornerRefinementMaxIterations(cornerRefinementMaxIterations)
    public  void set_cornerRefinementMaxIterations(int cornerRefinementMaxIterations)
    {
        
        set_cornerRefinementMaxIterations_0(nativeObj, cornerRefinementMaxIterations);
        
        return;
    }


    //
    // C++: double DetectorParameters::cornerRefinementMinAccuracy
    //

    //javadoc: DetectorParameters::get_cornerRefinementMinAccuracy()
    public  double get_cornerRefinementMinAccuracy()
    {
        
        double retVal = get_cornerRefinementMinAccuracy_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::cornerRefinementMinAccuracy
    //

    //javadoc: DetectorParameters::set_cornerRefinementMinAccuracy(cornerRefinementMinAccuracy)
    public  void set_cornerRefinementMinAccuracy(double cornerRefinementMinAccuracy)
    {
        
        set_cornerRefinementMinAccuracy_0(nativeObj, cornerRefinementMinAccuracy);
        
        return;
    }


    //
    // C++: int DetectorParameters::markerBorderBits
    //

    //javadoc: DetectorParameters::get_markerBorderBits()
    public  int get_markerBorderBits()
    {
        
        int retVal = get_markerBorderBits_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::markerBorderBits
    //

    //javadoc: DetectorParameters::set_markerBorderBits(markerBorderBits)
    public  void set_markerBorderBits(int markerBorderBits)
    {
        
        set_markerBorderBits_0(nativeObj, markerBorderBits);
        
        return;
    }


    //
    // C++: int DetectorParameters::perspectiveRemovePixelPerCell
    //

    //javadoc: DetectorParameters::get_perspectiveRemovePixelPerCell()
    public  int get_perspectiveRemovePixelPerCell()
    {
        
        int retVal = get_perspectiveRemovePixelPerCell_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::perspectiveRemovePixelPerCell
    //

    //javadoc: DetectorParameters::set_perspectiveRemovePixelPerCell(perspectiveRemovePixelPerCell)
    public  void set_perspectiveRemovePixelPerCell(int perspectiveRemovePixelPerCell)
    {
        
        set_perspectiveRemovePixelPerCell_0(nativeObj, perspectiveRemovePixelPerCell);
        
        return;
    }


    //
    // C++: double DetectorParameters::perspectiveRemoveIgnoredMarginPerCell
    //

    //javadoc: DetectorParameters::get_perspectiveRemoveIgnoredMarginPerCell()
    public  double get_perspectiveRemoveIgnoredMarginPerCell()
    {
        
        double retVal = get_perspectiveRemoveIgnoredMarginPerCell_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::perspectiveRemoveIgnoredMarginPerCell
    //

    //javadoc: DetectorParameters::set_perspectiveRemoveIgnoredMarginPerCell(perspectiveRemoveIgnoredMarginPerCell)
    public  void set_perspectiveRemoveIgnoredMarginPerCell(double perspectiveRemoveIgnoredMarginPerCell)
    {
        
        set_perspectiveRemoveIgnoredMarginPerCell_0(nativeObj, perspectiveRemoveIgnoredMarginPerCell);
        
        return;
    }


    //
    // C++: double DetectorParameters::maxErroneousBitsInBorderRate
    //

    //javadoc: DetectorParameters::get_maxErroneousBitsInBorderRate()
    public  double get_maxErroneousBitsInBorderRate()
    {
        
        double retVal = get_maxErroneousBitsInBorderRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::maxErroneousBitsInBorderRate
    //

    //javadoc: DetectorParameters::set_maxErroneousBitsInBorderRate(maxErroneousBitsInBorderRate)
    public  void set_maxErroneousBitsInBorderRate(double maxErroneousBitsInBorderRate)
    {
        
        set_maxErroneousBitsInBorderRate_0(nativeObj, maxErroneousBitsInBorderRate);
        
        return;
    }


    //
    // C++: double DetectorParameters::minOtsuStdDev
    //

    //javadoc: DetectorParameters::get_minOtsuStdDev()
    public  double get_minOtsuStdDev()
    {
        
        double retVal = get_minOtsuStdDev_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::minOtsuStdDev
    //

    //javadoc: DetectorParameters::set_minOtsuStdDev(minOtsuStdDev)
    public  void set_minOtsuStdDev(double minOtsuStdDev)
    {
        
        set_minOtsuStdDev_0(nativeObj, minOtsuStdDev);
        
        return;
    }


    //
    // C++: double DetectorParameters::errorCorrectionRate
    //

    //javadoc: DetectorParameters::get_errorCorrectionRate()
    public  double get_errorCorrectionRate()
    {
        
        double retVal = get_errorCorrectionRate_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void DetectorParameters::errorCorrectionRate
    //

    //javadoc: DetectorParameters::set_errorCorrectionRate(errorCorrectionRate)
    public  void set_errorCorrectionRate(double errorCorrectionRate)
    {
        
        set_errorCorrectionRate_0(nativeObj, errorCorrectionRate);
        
        return;
    }


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Ptr_DetectorParameters create()
    private static native long create_0();

    // C++: int DetectorParameters::adaptiveThreshWinSizeMin
    private static native int get_adaptiveThreshWinSizeMin_0(long nativeObj);

    // C++: void DetectorParameters::adaptiveThreshWinSizeMin
    private static native void set_adaptiveThreshWinSizeMin_0(long nativeObj, int adaptiveThreshWinSizeMin);

    // C++: int DetectorParameters::adaptiveThreshWinSizeMax
    private static native int get_adaptiveThreshWinSizeMax_0(long nativeObj);

    // C++: void DetectorParameters::adaptiveThreshWinSizeMax
    private static native void set_adaptiveThreshWinSizeMax_0(long nativeObj, int adaptiveThreshWinSizeMax);

    // C++: int DetectorParameters::adaptiveThreshWinSizeStep
    private static native int get_adaptiveThreshWinSizeStep_0(long nativeObj);

    // C++: void DetectorParameters::adaptiveThreshWinSizeStep
    private static native void set_adaptiveThreshWinSizeStep_0(long nativeObj, int adaptiveThreshWinSizeStep);

    // C++: double DetectorParameters::adaptiveThreshConstant
    private static native double get_adaptiveThreshConstant_0(long nativeObj);

    // C++: void DetectorParameters::adaptiveThreshConstant
    private static native void set_adaptiveThreshConstant_0(long nativeObj, double adaptiveThreshConstant);

    // C++: double DetectorParameters::minMarkerPerimeterRate
    private static native double get_minMarkerPerimeterRate_0(long nativeObj);

    // C++: void DetectorParameters::minMarkerPerimeterRate
    private static native void set_minMarkerPerimeterRate_0(long nativeObj, double minMarkerPerimeterRate);

    // C++: double DetectorParameters::maxMarkerPerimeterRate
    private static native double get_maxMarkerPerimeterRate_0(long nativeObj);

    // C++: void DetectorParameters::maxMarkerPerimeterRate
    private static native void set_maxMarkerPerimeterRate_0(long nativeObj, double maxMarkerPerimeterRate);

    // C++: double DetectorParameters::polygonalApproxAccuracyRate
    private static native double get_polygonalApproxAccuracyRate_0(long nativeObj);

    // C++: void DetectorParameters::polygonalApproxAccuracyRate
    private static native void set_polygonalApproxAccuracyRate_0(long nativeObj, double polygonalApproxAccuracyRate);

    // C++: double DetectorParameters::minCornerDistanceRate
    private static native double get_minCornerDistanceRate_0(long nativeObj);

    // C++: void DetectorParameters::minCornerDistanceRate
    private static native void set_minCornerDistanceRate_0(long nativeObj, double minCornerDistanceRate);

    // C++: int DetectorParameters::minDistanceToBorder
    private static native int get_minDistanceToBorder_0(long nativeObj);

    // C++: void DetectorParameters::minDistanceToBorder
    private static native void set_minDistanceToBorder_0(long nativeObj, int minDistanceToBorder);

    // C++: double DetectorParameters::minMarkerDistanceRate
    private static native double get_minMarkerDistanceRate_0(long nativeObj);

    // C++: void DetectorParameters::minMarkerDistanceRate
    private static native void set_minMarkerDistanceRate_0(long nativeObj, double minMarkerDistanceRate);

    // C++: int DetectorParameters::cornerRefinementMethod
    private static native int get_cornerRefinementMethod_0(long nativeObj);

    // C++: void DetectorParameters::cornerRefinementMethod
    private static native void set_cornerRefinementMethod_0(long nativeObj, int cornerRefinementMethod);

    // C++: int DetectorParameters::cornerRefinementWinSize
    private static native int get_cornerRefinementWinSize_0(long nativeObj);

    // C++: void DetectorParameters::cornerRefinementWinSize
    private static native void set_cornerRefinementWinSize_0(long nativeObj, int cornerRefinementWinSize);

    // C++: int DetectorParameters::cornerRefinementMaxIterations
    private static native int get_cornerRefinementMaxIterations_0(long nativeObj);

    // C++: void DetectorParameters::cornerRefinementMaxIterations
    private static native void set_cornerRefinementMaxIterations_0(long nativeObj, int cornerRefinementMaxIterations);

    // C++: double DetectorParameters::cornerRefinementMinAccuracy
    private static native double get_cornerRefinementMinAccuracy_0(long nativeObj);

    // C++: void DetectorParameters::cornerRefinementMinAccuracy
    private static native void set_cornerRefinementMinAccuracy_0(long nativeObj, double cornerRefinementMinAccuracy);

    // C++: int DetectorParameters::markerBorderBits
    private static native int get_markerBorderBits_0(long nativeObj);

    // C++: void DetectorParameters::markerBorderBits
    private static native void set_markerBorderBits_0(long nativeObj, int markerBorderBits);

    // C++: int DetectorParameters::perspectiveRemovePixelPerCell
    private static native int get_perspectiveRemovePixelPerCell_0(long nativeObj);

    // C++: void DetectorParameters::perspectiveRemovePixelPerCell
    private static native void set_perspectiveRemovePixelPerCell_0(long nativeObj, int perspectiveRemovePixelPerCell);

    // C++: double DetectorParameters::perspectiveRemoveIgnoredMarginPerCell
    private static native double get_perspectiveRemoveIgnoredMarginPerCell_0(long nativeObj);

    // C++: void DetectorParameters::perspectiveRemoveIgnoredMarginPerCell
    private static native void set_perspectiveRemoveIgnoredMarginPerCell_0(long nativeObj, double perspectiveRemoveIgnoredMarginPerCell);

    // C++: double DetectorParameters::maxErroneousBitsInBorderRate
    private static native double get_maxErroneousBitsInBorderRate_0(long nativeObj);

    // C++: void DetectorParameters::maxErroneousBitsInBorderRate
    private static native void set_maxErroneousBitsInBorderRate_0(long nativeObj, double maxErroneousBitsInBorderRate);

    // C++: double DetectorParameters::minOtsuStdDev
    private static native double get_minOtsuStdDev_0(long nativeObj);

    // C++: void DetectorParameters::minOtsuStdDev
    private static native void set_minOtsuStdDev_0(long nativeObj, double minOtsuStdDev);

    // C++: double DetectorParameters::errorCorrectionRate
    private static native double get_errorCorrectionRate_0(long nativeObj);

    // C++: void DetectorParameters::errorCorrectionRate
    private static native void set_errorCorrectionRate_0(long nativeObj, double errorCorrectionRate);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
