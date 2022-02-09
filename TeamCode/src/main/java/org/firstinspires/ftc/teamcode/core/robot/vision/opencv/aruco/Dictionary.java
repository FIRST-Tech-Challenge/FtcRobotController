//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Mat;

// C++: class Dictionary
//javadoc: Dictionary

public class Dictionary {

    protected final long nativeObj;
    protected Dictionary(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    // internal usage only
    public static Dictionary __fromPtr__(long addr) { return new Dictionary(addr); }

    //
    // C++: static Mat getBitsFromByteList(Mat byteList, int markerSize)
    //

    //javadoc: Dictionary::getBitsFromByteList(byteList, markerSize)
    public static Mat getBitsFromByteList(Mat byteList, int markerSize)
    {
        
        Mat retVal = new Mat(getBitsFromByteList_0(byteList.nativeObj, markerSize));
        
        return retVal;
    }


    //
    // C++: static Mat getByteListFromBits(Mat bits)
    //

    //javadoc: Dictionary::getByteListFromBits(bits)
    public static Mat getByteListFromBits(Mat bits)
    {
        
        Mat retVal = new Mat(getByteListFromBits_0(bits.nativeObj));
        
        return retVal;
    }


    //
    // C++: static Ptr_Dictionary create(int nMarkers, int markerSize, Ptr_Dictionary baseDictionary)
    //

    //javadoc: Dictionary::create(nMarkers, markerSize, baseDictionary)
    public static Dictionary create_from(int nMarkers, int markerSize, Dictionary baseDictionary)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(create_from_0(nMarkers, markerSize, baseDictionary.getNativeObjAddr()));
        
        return retVal;
    }


    //
    // C++: static Ptr_Dictionary create(int nMarkers, int markerSize)
    //

    //javadoc: Dictionary::create(nMarkers, markerSize)
    public static Dictionary create(int nMarkers, int markerSize)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(create_0(nMarkers, markerSize));
        
        return retVal;
    }


    //
    // C++: static Ptr_Dictionary get(int dict)
    //

    //javadoc: Dictionary::get(dict)
    public static Dictionary get(int dict)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(get_0(dict));
        
        return retVal;
    }


    //
    // C++:  void drawMarker(int id, int sidePixels, Mat& _img, int borderBits = 1)
    //

    //javadoc: Dictionary::drawMarker(id, sidePixels, _img, borderBits)
    public  void drawMarker(int id, int sidePixels, Mat _img, int borderBits)
    {
        
        drawMarker_0(nativeObj, id, sidePixels, _img.nativeObj, borderBits);
        
        return;
    }

    //javadoc: Dictionary::drawMarker(id, sidePixels, _img)
    public  void drawMarker(int id, int sidePixels, Mat _img)
    {
        
        drawMarker_1(nativeObj, id, sidePixels, _img.nativeObj);
        
        return;
    }


    //
    // C++: Mat Dictionary::bytesList
    //

    //javadoc: Dictionary::get_bytesList()
    public  Mat get_bytesList()
    {
        
        Mat retVal = new Mat(get_bytesList_0(nativeObj));
        
        return retVal;
    }


    //
    // C++: void Dictionary::bytesList
    //

    //javadoc: Dictionary::set_bytesList(bytesList)
    public  void set_bytesList(Mat bytesList)
    {
        
        set_bytesList_0(nativeObj, bytesList.nativeObj);
        
        return;
    }


    //
    // C++: int Dictionary::markerSize
    //

    //javadoc: Dictionary::get_markerSize()
    public  int get_markerSize()
    {
        
        int retVal = get_markerSize_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void Dictionary::markerSize
    //

    //javadoc: Dictionary::set_markerSize(markerSize)
    public  void set_markerSize(int markerSize)
    {
        
        set_markerSize_0(nativeObj, markerSize);
        
        return;
    }


    //
    // C++: int Dictionary::maxCorrectionBits
    //

    //javadoc: Dictionary::get_maxCorrectionBits()
    public  int get_maxCorrectionBits()
    {
        
        int retVal = get_maxCorrectionBits_0(nativeObj);
        
        return retVal;
    }


    //
    // C++: void Dictionary::maxCorrectionBits
    //

    //javadoc: Dictionary::set_maxCorrectionBits(maxCorrectionBits)
    public  void set_maxCorrectionBits(int maxCorrectionBits)
    {
        
        set_maxCorrectionBits_0(nativeObj, maxCorrectionBits);
        
        return;
    }


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Mat getBitsFromByteList(Mat byteList, int markerSize)
    private static native long getBitsFromByteList_0(long byteList_nativeObj, int markerSize);

    // C++: static Mat getByteListFromBits(Mat bits)
    private static native long getByteListFromBits_0(long bits_nativeObj);

    // C++: static Ptr_Dictionary create(int nMarkers, int markerSize, Ptr_Dictionary baseDictionary)
    private static native long create_from_0(int nMarkers, int markerSize, long baseDictionary_nativeObj);

    // C++: static Ptr_Dictionary create(int nMarkers, int markerSize)
    private static native long create_0(int nMarkers, int markerSize);

    // C++: static Ptr_Dictionary get(int dict)
    private static native long get_0(int dict);

    // C++:  void drawMarker(int id, int sidePixels, Mat& _img, int borderBits = 1)
    private static native void drawMarker_0(long nativeObj, int id, int sidePixels, long _img_nativeObj, int borderBits);
    private static native void drawMarker_1(long nativeObj, int id, int sidePixels, long _img_nativeObj);

    // C++: Mat Dictionary::bytesList
    private static native long get_bytesList_0(long nativeObj);

    // C++: void Dictionary::bytesList
    private static native void set_bytesList_0(long nativeObj, long bytesList_nativeObj);

    // C++: int Dictionary::markerSize
    private static native int get_markerSize_0(long nativeObj);

    // C++: void Dictionary::markerSize
    private static native void set_markerSize_0(long nativeObj, int markerSize);

    // C++: int Dictionary::maxCorrectionBits
    private static native int get_maxCorrectionBits_0(long nativeObj);

    // C++: void Dictionary::maxCorrectionBits
    private static native void set_maxCorrectionBits_0(long nativeObj, int maxCorrectionBits);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
