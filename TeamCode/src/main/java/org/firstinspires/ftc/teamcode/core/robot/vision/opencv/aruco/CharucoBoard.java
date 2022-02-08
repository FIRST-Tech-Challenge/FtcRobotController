//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Board;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.CharucoBoard;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Size;
import org.opencv.utils.Converters;

// C++: class CharucoBoard
//javadoc: CharucoBoard

public class CharucoBoard extends Board {

    protected CharucoBoard(long addr) { super(addr); }

    // internal usage only
    public static CharucoBoard __fromPtr__(long addr) { return new CharucoBoard(addr); }

    //
    // C++: static Ptr_CharucoBoard create(int squaresX, int squaresY, float squareLength, float markerLength, Ptr_Dictionary dictionary)
    //

    //javadoc: CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary)
    public static CharucoBoard create(int squaresX, int squaresY, float squareLength, float markerLength, Dictionary dictionary)
    {
        
        CharucoBoard retVal = CharucoBoard.__fromPtr__(create_0(squaresX, squaresY, squareLength, markerLength, dictionary.getNativeObjAddr()));
        
        return retVal;
    }


    //
    // C++:  Size getChessboardSize()
    //

    //javadoc: CharucoBoard::getChessboardSize()
    public  Size getChessboardSize()
    {
        
        Size retVal = new Size(getChessboardSize_0(nativeObj));
        
        return retVal;
    }


    //
    // C++:  float getMarkerLength()
    //

    //javadoc: CharucoBoard::getMarkerLength()
    public  float getMarkerLength()
    {
        
        float retVal = getMarkerLength_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  float getSquareLength()
    //

    //javadoc: CharucoBoard::getSquareLength()
    public  float getSquareLength()
    {
        
        float retVal = getSquareLength_0(nativeObj);
        
        return retVal;
    }


    //
    // C++:  void draw(Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    //

    //javadoc: CharucoBoard::draw(outSize, img, marginSize, borderBits)
    public  void draw(Size outSize, Mat img, int marginSize, int borderBits)
    {
        
        draw_0(nativeObj, outSize.width, outSize.height, img.nativeObj, marginSize, borderBits);
        
        return;
    }

    //javadoc: CharucoBoard::draw(outSize, img)
    public  void draw(Size outSize, Mat img)
    {
        
        draw_1(nativeObj, outSize.width, outSize.height, img.nativeObj);
        
        return;
    }


    //
    // C++: vector_Point3f CharucoBoard::chessboardCorners
    //

    //javadoc: CharucoBoard::get_chessboardCorners()
    public  MatOfPoint3f get_chessboardCorners()
    {
        
        MatOfPoint3f retVal = MatOfPoint3f.fromNativeAddr(get_chessboardCorners_0(nativeObj));
        
        return retVal;
    }


    //
    // C++: vector_vector_int CharucoBoard::nearestMarkerIdx
    //

    // Return type 'vector_vector_int' is not supported, skipping the function


    //
    // C++: vector_vector_int CharucoBoard::nearestMarkerCorners
    //

    // Return type 'vector_vector_int' is not supported, skipping the function


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Ptr_CharucoBoard create(int squaresX, int squaresY, float squareLength, float markerLength, Ptr_Dictionary dictionary)
    private static native long create_0(int squaresX, int squaresY, float squareLength, float markerLength, long dictionary_nativeObj);

    // C++:  Size getChessboardSize()
    private static native double[] getChessboardSize_0(long nativeObj);

    // C++:  float getMarkerLength()
    private static native float getMarkerLength_0(long nativeObj);

    // C++:  float getSquareLength()
    private static native float getSquareLength_0(long nativeObj);

    // C++:  void draw(Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    private static native void draw_0(long nativeObj, double outSize_width, double outSize_height, long img_nativeObj, int marginSize, int borderBits);
    private static native void draw_1(long nativeObj, double outSize_width, double outSize_height, long img_nativeObj);

    // C++: vector_Point3f CharucoBoard::chessboardCorners
    private static native long get_chessboardCorners_0(long nativeObj);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
