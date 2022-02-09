//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Board;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint3f;
import org.opencv.utils.Converters;

// C++: class Board
//javadoc: Board

public class Board {

    protected final long nativeObj;
    protected Board(long addr) { nativeObj = addr; }

    public long getNativeObjAddr() { return nativeObj; }

    // internal usage only
    public static Board __fromPtr__(long addr) { return new Board(addr); }

    //
    // C++: static Ptr_Board create(vector_Mat objPoints, Ptr_Dictionary dictionary, Mat ids)
    //

    //javadoc: Board::create(objPoints, dictionary, ids)
    public static Board create(List<Mat> objPoints, Dictionary dictionary, Mat ids)
    {
        Mat objPoints_mat = Converters.vector_Mat_to_Mat(objPoints);
        Board retVal = Board.__fromPtr__(create_0(objPoints_mat.nativeObj, dictionary.getNativeObjAddr(), ids.nativeObj));
        
        return retVal;
    }


    //
    // C++: vector_vector_Point3f Board::objPoints
    //

    //javadoc: Board::get_objPoints()
    public  List<MatOfPoint3f> get_objPoints()
    {
        List<MatOfPoint3f> retVal = new ArrayList<MatOfPoint3f>();
        Mat retValMat = new Mat(get_objPoints_0(nativeObj));
        Converters.Mat_to_vector_vector_Point3f(retValMat, retVal);
        return retVal;
    }


    //
    // C++: Ptr_Dictionary Board::dictionary
    //

    //javadoc: Board::get_dictionary()
    public  Dictionary get_dictionary()
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(get_dictionary_0(nativeObj));
        
        return retVal;
    }


    //
    // C++: vector_int Board::ids
    //

    //javadoc: Board::get_ids()
    public  MatOfInt get_ids()
    {
        
        MatOfInt retVal = MatOfInt.fromNativeAddr(get_ids_0(nativeObj));
        
        return retVal;
    }


    @Override
    protected void finalize() throws Throwable {
        delete(nativeObj);
    }



    // C++: static Ptr_Board create(vector_Mat objPoints, Ptr_Dictionary dictionary, Mat ids)
    private static native long create_0(long objPoints_mat_nativeObj, long dictionary_nativeObj, long ids_nativeObj);

    // C++: vector_vector_Point3f Board::objPoints
    private static native long get_objPoints_0(long nativeObj);

    // C++: Ptr_Dictionary Board::dictionary
    private static native long get_dictionary_0(long nativeObj);

    // C++: vector_int Board::ids
    private static native long get_ids_0(long nativeObj);

    // native support for java finalize()
    private static native void delete(long nativeObj);

}
