//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Board;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.GridBoard;
import org.opencv.core.Mat;
import org.opencv.core.Size;

// C++: class GridBoard
//javadoc: GridBoard

public class GridBoard extends Board {

    protected GridBoard(long addr) { super(addr); }

    // internal usage only
    public static GridBoard __fromPtr__(long addr) { return new GridBoard(addr); }

    //
    // C++: static Ptr_GridBoard create(int markersX, int markersY, float markerLength, float markerSeparation, Ptr_Dictionary dictionary, int firstMarker = 0)
    //

    //javadoc: GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary, firstMarker)
    public static GridBoard create(int markersX, int markersY, float markerLength, float markerSeparation, Dictionary dictionary, int firstMarker)
    {
        
        GridBoard retVal = GridBoard.__fromPtr__(create_0(markersX, markersY, markerLength, markerSeparation, dictionary.getNativeObjAddr(), firstMarker));
        
        return retVal;
    }

    //javadoc: GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary)
    public static GridBoard create(int markersX, int markersY, float markerLength, float markerSeparation, Dictionary dictionary)
    {
        
        GridBoard retVal = GridBoard.__fromPtr__(create_1(markersX, markersY, markerLength, markerSeparation, dictionary.getNativeObjAddr()));
        
        return retVal;
    }


    //
    // C++:  Size getGridSize()
    //

    //javadoc: GridBoard::getGridSize()
    public  Size getGridSize()
    {
        
        Size retVal = new Size(getGridSize_0(nativeObj));
        
        return retVal;
    }


    //
    // C++:  float getMarkerLength()
    //

    //javadoc: GridBoard::getMarkerLength()
    public  float getMarkerLength()
    {
        
        float retVal = getMarkerLength_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  float getMarkerSeparation()
    //

    //javadoc: GridBoard::getMarkerSeparation()
    public  float getMarkerSeparation()
    {
        
        float retVal = getMarkerSeparation_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  void draw(Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    //

    //javadoc: GridBoard::draw(outSize, img, marginSize, borderBits)
    public  void draw(Size outSize, Mat img, int marginSize, int borderBits)
    {
        
        draw_0(nativeObj, outSize.width, outSize.height, img.nativeObj, marginSize, borderBits);
        
        return;
    }

    //javadoc: GridBoard::draw(outSize, img)
    public  void draw(Size outSize, Mat img)
    {
        
        draw_1(nativeObj, outSize.width, outSize.height, img.nativeObj);
        
        return;
    }


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Ptr_GridBoard create(int markersX, int markersY, float markerLength, float markerSeparation, Ptr_Dictionary dictionary, int firstMarker = 0)
    private static native long create_0(int markersX, int markersY, float markerLength, float markerSeparation, long dictionary_nativeObj, int firstMarker);
    private static native long create_1(int markersX, int markersY, float markerLength, float markerSeparation, long dictionary_nativeObj);

    // C++:  Size getGridSize()
    private static native double[] getGridSize_0(long nativeObj);

    // C++:  float getMarkerLength()
    private static native float getMarkerLength_0(long nativeObj);

    // C++:  float getMarkerSeparation()
    private static native float getMarkerSeparation_0(long nativeObj);

    // C++:  void draw(Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    private static native void draw_0(long nativeObj, double outSize_width, double outSize_height, long img_nativeObj, int marginSize, int borderBits);
    private static native void draw_1(long nativeObj, double outSize_width, double outSize_height, long img_nativeObj);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
