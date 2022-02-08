//
// This file is auto-generated. Please don't modify it!
//
package org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Board;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.CharucoBoard;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.DetectorParameters;
import org.firstinspires.ftc.teamcode.core.robot.vision.opencv.aruco.Dictionary;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.TermCriteria;
import org.opencv.utils.Converters;

// C++: class Aruco
//javadoc: Aruco

public class Aruco {

    public static final int
            CORNER_REFINE_NONE = 0,
            CORNER_REFINE_SUBPIX = 1,
            CORNER_REFINE_CONTOUR = 2,
            DICT_4X4_50 = 0,
            DICT_4X4_100 = 0+1,
            DICT_4X4_250 = 0+2,
            DICT_4X4_1000 = 0+3,
            DICT_5X5_50 = 0+4,
            DICT_5X5_100 = 0+5,
            DICT_5X5_250 = 0+6,
            DICT_5X5_1000 = 0+7,
            DICT_6X6_50 = 0+8,
            DICT_6X6_100 = 0+9,
            DICT_6X6_250 = 0+10,
            DICT_6X6_1000 = 0+11,
            DICT_7X7_50 = 0+12,
            DICT_7X7_100 = 0+13,
            DICT_7X7_250 = 0+14,
            DICT_7X7_1000 = 0+15,
            DICT_ARUCO_ORIGINAL = 0+16;


    //
    // C++:  Ptr_Dictionary generateCustomDictionary(int nMarkers, int markerSize, Ptr_Dictionary baseDictionary)
    //

    //javadoc: generateCustomDictionary(nMarkers, markerSize, baseDictionary)
    public static Dictionary custom_dictionary_from(int nMarkers, int markerSize, Dictionary baseDictionary)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(custom_dictionary_from_0(nMarkers, markerSize, baseDictionary.getNativeObjAddr()));
        
        return retVal;
    }


    //
    // C++:  Ptr_Dictionary generateCustomDictionary(int nMarkers, int markerSize)
    //

    //javadoc: generateCustomDictionary(nMarkers, markerSize)
    public static Dictionary custom_dictionary(int nMarkers, int markerSize)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(custom_dictionary_0(nMarkers, markerSize));
        
        return retVal;
    }


    //
    // C++:  Ptr_Dictionary getPredefinedDictionary(int dict)
    //

    //javadoc: getPredefinedDictionary(dict)
    public static Dictionary getPredefinedDictionary(int dict)
    {
        
        Dictionary retVal = Dictionary.__fromPtr__(getPredefinedDictionary_0(dict));
        
        return retVal;
    }


    //
    // C++:  bool estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, Ptr_CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false)
    //

    //javadoc: estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess)
    public static boolean estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess)
    {
        
        boolean retVal = estimatePoseCharucoBoard_0(charucoCorners.nativeObj, charucoIds.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
        
        return retVal;
    }

    //javadoc: estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec)
    public static boolean estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
    {
        
        boolean retVal = estimatePoseCharucoBoard_1(charucoCorners.nativeObj, charucoIds.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
        
        return retVal;
    }


    //
    // C++:  double calibrateCameraAruco(vector_Mat corners, Mat ids, Mat counter, Ptr_Board board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    //

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, flags, criteria)
    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags, TermCriteria criteria)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraArucoExtended_0(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, flags)
    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraArucoExtended_1(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors)
    public static double calibrateCameraArucoExtended(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraArucoExtended_2(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double calibrateCameraAruco(vector_Mat corners, Mat ids, Mat counter, Ptr_Board board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs = vector_Mat(), vector_Mat& tvecs = vector_Mat(), int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    //

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags, criteria)
    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraAruco_0(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags)
    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraAruco_1(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraAruco(corners, ids, counter, board, imageSize, cameraMatrix, distCoeffs)
    public static double calibrateCameraAruco(List<Mat> corners, Mat ids, Mat counter, Board board, Size imageSize, Mat cameraMatrix, Mat distCoeffs)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        double retVal = calibrateCameraAruco_2(corners_mat.nativeObj, ids.nativeObj, counter.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj);
        
        return retVal;
    }


    //
    // C++:  double calibrateCameraCharuco(vector_Mat charucoCorners, vector_Mat charucoIds, Ptr_CharucoBoard board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    //

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, flags, criteria)
    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags, TermCriteria criteria)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraCharucoExtended_0(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors, flags)
    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors, int flags)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraCharucoExtended_1(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors)
    public static double calibrateCameraCharucoExtended(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, Mat stdDeviationsIntrinsics, Mat stdDeviationsExtrinsics, Mat perViewErrors)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraCharucoExtended_2(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, stdDeviationsIntrinsics.nativeObj, stdDeviationsExtrinsics.nativeObj, perViewErrors.nativeObj);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }


    //
    // C++:  double calibrateCameraCharuco(vector_Mat charucoCorners, vector_Mat charucoIds, Ptr_CharucoBoard board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs = vector_Mat(), vector_Mat& tvecs = vector_Mat(), int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    //

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags, criteria)
    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags, TermCriteria criteria)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraCharuco_0(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags, criteria.type, criteria.maxCount, criteria.epsilon);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags)
    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs, List<Mat> rvecs, List<Mat> tvecs, int flags)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        Mat rvecs_mat = new Mat();
        Mat tvecs_mat = new Mat();
        double retVal = calibrateCameraCharuco_1(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs_mat.nativeObj, tvecs_mat.nativeObj, flags);
        Converters.Mat_to_vector_Mat(rvecs_mat, rvecs);
        rvecs_mat.release();
        Converters.Mat_to_vector_Mat(tvecs_mat, tvecs);
        tvecs_mat.release();
        return retVal;
    }

    //javadoc: calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs)
    public static double calibrateCameraCharuco(List<Mat> charucoCorners, List<Mat> charucoIds, CharucoBoard board, Size imageSize, Mat cameraMatrix, Mat distCoeffs)
    {
        Mat charucoCorners_mat = Converters.vector_Mat_to_Mat(charucoCorners);
        Mat charucoIds_mat = Converters.vector_Mat_to_Mat(charucoIds);
        double retVal = calibrateCameraCharuco_2(charucoCorners_mat.nativeObj, charucoIds_mat.nativeObj, board.getNativeObjAddr(), imageSize.width, imageSize.height, cameraMatrix.nativeObj, distCoeffs.nativeObj);
        
        return retVal;
    }


    //
    // C++:  int estimatePoseBoard(vector_Mat corners, Mat ids, Ptr_Board board, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false)
    //

    //javadoc: estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec, useExtrinsicGuess)
    public static int estimatePoseBoard(List<Mat> corners, Mat ids, Board board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, boolean useExtrinsicGuess)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        int retVal = estimatePoseBoard_0(corners_mat.nativeObj, ids.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, useExtrinsicGuess);
        
        return retVal;
    }

    //javadoc: estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec, tvec)
    public static int estimatePoseBoard(List<Mat> corners, Mat ids, Board board, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        int retVal = estimatePoseBoard_1(corners_mat.nativeObj, ids.nativeObj, board.getNativeObjAddr(), cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj);
        
        return retVal;
    }


    //
    // C++:  int interpolateCornersCharuco(vector_Mat markerCorners, Mat markerIds, Mat image, Ptr_CharucoBoard board, Mat& charucoCorners, Mat& charucoIds, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat(), int minMarkers = 2)
    //

    //javadoc: interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs, minMarkers)
    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds, Mat cameraMatrix, Mat distCoeffs, int minMarkers)
    {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        int retVal = interpolateCornersCharuco_0(markerCorners_mat.nativeObj, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minMarkers);
        
        return retVal;
    }

    //javadoc: interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds)
    public static int interpolateCornersCharuco(List<Mat> markerCorners, Mat markerIds, Mat image, CharucoBoard board, Mat charucoCorners, Mat charucoIds)
    {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        int retVal = interpolateCornersCharuco_1(markerCorners_mat.nativeObj, markerIds.nativeObj, image.nativeObj, board.getNativeObjAddr(), charucoCorners.nativeObj, charucoIds.nativeObj);
        
        return retVal;
    }


    //
    // C++:  void detectCharucoDiamond(Mat image, vector_Mat markerCorners, Mat markerIds, float squareMarkerLengthRate, vector_Mat& diamondCorners, Mat& diamondIds, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat())
    //

    //javadoc: detectCharucoDiamond(image, markerCorners, markerIds, squareMarkerLengthRate, diamondCorners, diamondIds, cameraMatrix, distCoeffs)
    public static void detectCharucoDiamond(Mat image, List<Mat> markerCorners, Mat markerIds, float squareMarkerLengthRate, List<Mat> diamondCorners, Mat diamondIds, Mat cameraMatrix, Mat distCoeffs)
    {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat diamondCorners_mat = new Mat();
        detectCharucoDiamond_0(image.nativeObj, markerCorners_mat.nativeObj, markerIds.nativeObj, squareMarkerLengthRate, diamondCorners_mat.nativeObj, diamondIds.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj);
        Converters.Mat_to_vector_Mat(diamondCorners_mat, diamondCorners);
        diamondCorners_mat.release();
        return;
    }

    //javadoc: detectCharucoDiamond(image, markerCorners, markerIds, squareMarkerLengthRate, diamondCorners, diamondIds)
    public static void detectCharucoDiamond(Mat image, List<Mat> markerCorners, Mat markerIds, float squareMarkerLengthRate, List<Mat> diamondCorners, Mat diamondIds)
    {
        Mat markerCorners_mat = Converters.vector_Mat_to_Mat(markerCorners);
        Mat diamondCorners_mat = new Mat();
        detectCharucoDiamond_1(image.nativeObj, markerCorners_mat.nativeObj, markerIds.nativeObj, squareMarkerLengthRate, diamondCorners_mat.nativeObj, diamondIds.nativeObj);
        Converters.Mat_to_vector_Mat(diamondCorners_mat, diamondCorners);
        diamondCorners_mat.release();
        return;
    }


    //
    // C++:  void detectMarkers(Mat image, Ptr_Dictionary dictionary, vector_Mat& corners, Mat& ids, Ptr_DetectorParameters parameters = DetectorParameters::create(), vector_Mat& rejectedImgPoints = vector_Mat(), Mat cameraMatrix = Mat(), Mat distCoeff = Mat())
    //

    //javadoc: detectMarkers(image, dictionary, corners, ids, parameters, rejectedImgPoints, cameraMatrix, distCoeff)
    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids, DetectorParameters parameters, List<Mat> rejectedImgPoints, Mat cameraMatrix, Mat distCoeff)
    {
        Mat corners_mat = new Mat();
        Mat rejectedImgPoints_mat = new Mat();
        detectMarkers_0(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj, parameters.getNativeObjAddr(), rejectedImgPoints_mat.nativeObj, cameraMatrix.nativeObj, distCoeff.nativeObj);
        Converters.Mat_to_vector_Mat(corners_mat, corners);
        corners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedImgPoints_mat, rejectedImgPoints);
        rejectedImgPoints_mat.release();
        return;
    }

    //javadoc: detectMarkers(image, dictionary, corners, ids)
    public static void detectMarkers(Mat image, Dictionary dictionary, List<Mat> corners, Mat ids)
    {
        Mat corners_mat = new Mat();
        detectMarkers_1(image.nativeObj, dictionary.getNativeObjAddr(), corners_mat.nativeObj, ids.nativeObj);
        Converters.Mat_to_vector_Mat(corners_mat, corners);
        corners_mat.release();
        return;
    }


    //
    // C++:  void drawAxis(Mat& image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length)
    //

    //javadoc: drawAxis(image, cameraMatrix, distCoeffs, rvec, tvec, length)
    public static void drawAxis(Mat image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length)
    {
        
        drawAxis_0(image.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvec.nativeObj, tvec.nativeObj, length);
        
        return;
    }


    //
    // C++:  void drawDetectedCornersCharuco(Mat& image, Mat charucoCorners, Mat charucoIds = Mat(), Scalar cornerColor = Scalar(255, 0, 0))
    //

    //javadoc: drawDetectedCornersCharuco(image, charucoCorners, charucoIds, cornerColor)
    public static void drawDetectedCornersCharuco(Mat image, Mat charucoCorners, Mat charucoIds, Scalar cornerColor)
    {
        
        drawDetectedCornersCharuco_0(image.nativeObj, charucoCorners.nativeObj, charucoIds.nativeObj, cornerColor.val[0], cornerColor.val[1], cornerColor.val[2], cornerColor.val[3]);
        
        return;
    }

    //javadoc: drawDetectedCornersCharuco(image, charucoCorners)
    public static void drawDetectedCornersCharuco(Mat image, Mat charucoCorners)
    {
        
        drawDetectedCornersCharuco_1(image.nativeObj, charucoCorners.nativeObj);
        
        return;
    }


    //
    // C++:  void drawDetectedDiamonds(Mat& image, vector_Mat diamondCorners, Mat diamondIds = Mat(), Scalar borderColor = Scalar(0, 0, 255))
    //

    //javadoc: drawDetectedDiamonds(image, diamondCorners, diamondIds, borderColor)
    public static void drawDetectedDiamonds(Mat image, List<Mat> diamondCorners, Mat diamondIds, Scalar borderColor)
    {
        Mat diamondCorners_mat = Converters.vector_Mat_to_Mat(diamondCorners);
        drawDetectedDiamonds_0(image.nativeObj, diamondCorners_mat.nativeObj, diamondIds.nativeObj, borderColor.val[0], borderColor.val[1], borderColor.val[2], borderColor.val[3]);
        
        return;
    }

    //javadoc: drawDetectedDiamonds(image, diamondCorners)
    public static void drawDetectedDiamonds(Mat image, List<Mat> diamondCorners)
    {
        Mat diamondCorners_mat = Converters.vector_Mat_to_Mat(diamondCorners);
        drawDetectedDiamonds_1(image.nativeObj, diamondCorners_mat.nativeObj);
        
        return;
    }


    //
    // C++:  void drawDetectedMarkers(Mat& image, vector_Mat corners, Mat ids = Mat(), Scalar borderColor = Scalar(0, 255, 0))
    //

    //javadoc: drawDetectedMarkers(image, corners, ids, borderColor)
    public static void drawDetectedMarkers(Mat image, List<Mat> corners, Mat ids, Scalar borderColor)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        drawDetectedMarkers_0(image.nativeObj, corners_mat.nativeObj, ids.nativeObj, borderColor.val[0], borderColor.val[1], borderColor.val[2], borderColor.val[3]);
        
        return;
    }

    //javadoc: drawDetectedMarkers(image, corners)
    public static void drawDetectedMarkers(Mat image, List<Mat> corners)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        drawDetectedMarkers_1(image.nativeObj, corners_mat.nativeObj);
        
        return;
    }


    //
    // C++:  void drawMarker(Ptr_Dictionary dictionary, int id, int sidePixels, Mat& img, int borderBits = 1)
    //

    //javadoc: drawMarker(dictionary, id, sidePixels, img, borderBits)
    public static void drawMarker(Dictionary dictionary, int id, int sidePixels, Mat img, int borderBits)
    {
        
        drawMarker_0(dictionary.getNativeObjAddr(), id, sidePixels, img.nativeObj, borderBits);
        
        return;
    }

    //javadoc: drawMarker(dictionary, id, sidePixels, img)
    public static void drawMarker(Dictionary dictionary, int id, int sidePixels, Mat img)
    {
        
        drawMarker_1(dictionary.getNativeObjAddr(), id, sidePixels, img.nativeObj);
        
        return;
    }


    //
    // C++:  void drawPlanarBoard(Ptr_Board board, Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    //

    //javadoc: drawPlanarBoard(board, outSize, img, marginSize, borderBits)
    public static void drawPlanarBoard(Board board, Size outSize, Mat img, int marginSize, int borderBits)
    {
        
        drawPlanarBoard_0(board.getNativeObjAddr(), outSize.width, outSize.height, img.nativeObj, marginSize, borderBits);
        
        return;
    }

    //javadoc: drawPlanarBoard(board, outSize, img)
    public static void drawPlanarBoard(Board board, Size outSize, Mat img)
    {
        
        drawPlanarBoard_1(board.getNativeObjAddr(), outSize.width, outSize.height, img.nativeObj);
        
        return;
    }


    //
    // C++:  void estimatePoseSingleMarkers(vector_Mat corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat& rvecs, Mat& tvecs, Mat& _objPoints = Mat())
    //

    //javadoc: estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs, _objPoints)
    public static void estimatePoseSingleMarkers(List<Mat> corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat rvecs, Mat tvecs, Mat _objPoints)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        estimatePoseSingleMarkers_0(corners_mat.nativeObj, markerLength, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs.nativeObj, tvecs.nativeObj, _objPoints.nativeObj);
        
        return;
    }

    //javadoc: estimatePoseSingleMarkers(corners, markerLength, cameraMatrix, distCoeffs, rvecs, tvecs)
    public static void estimatePoseSingleMarkers(List<Mat> corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat rvecs, Mat tvecs)
    {
        Mat corners_mat = Converters.vector_Mat_to_Mat(corners);
        estimatePoseSingleMarkers_1(corners_mat.nativeObj, markerLength, cameraMatrix.nativeObj, distCoeffs.nativeObj, rvecs.nativeObj, tvecs.nativeObj);
        
        return;
    }


    //
    // C++:  void getBoardObjectAndImagePoints(Ptr_Board board, vector_Mat detectedCorners, Mat detectedIds, Mat& objPoints, Mat& imgPoints)
    //

    //javadoc: getBoardObjectAndImagePoints(board, detectedCorners, detectedIds, objPoints, imgPoints)
    public static void getBoardObjectAndImagePoints(Board board, List<Mat> detectedCorners, Mat detectedIds, Mat objPoints, Mat imgPoints)
    {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        getBoardObjectAndImagePoints_0(board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, objPoints.nativeObj, imgPoints.nativeObj);
        
        return;
    }


    //
    // C++:  void refineDetectedMarkers(Mat image, Ptr_Board board, vector_Mat& detectedCorners, Mat& detectedIds, vector_Mat& rejectedCorners, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat(), float minRepDistance = 10.f, float errorCorrectionRate = 3.f, bool checkAllOrders = true, Mat& recoveredIdxs = Mat(), Ptr_DetectorParameters parameters = DetectorParameters::create())
    //

    //javadoc: refineDetectedMarkers(image, board, detectedCorners, detectedIds, rejectedCorners, cameraMatrix, distCoeffs, minRepDistance, errorCorrectionRate, checkAllOrders, recoveredIdxs, parameters)
    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners, Mat cameraMatrix, Mat distCoeffs, float minRepDistance, float errorCorrectionRate, boolean checkAllOrders, Mat recoveredIdxs, DetectorParameters parameters)
    {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        refineDetectedMarkers_0(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj, cameraMatrix.nativeObj, distCoeffs.nativeObj, minRepDistance, errorCorrectionRate, checkAllOrders, recoveredIdxs.nativeObj, parameters.getNativeObjAddr());
        Converters.Mat_to_vector_Mat(detectedCorners_mat, detectedCorners);
        detectedCorners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedCorners_mat, rejectedCorners);
        rejectedCorners_mat.release();
        return;
    }

    //javadoc: refineDetectedMarkers(image, board, detectedCorners, detectedIds, rejectedCorners)
    public static void refineDetectedMarkers(Mat image, Board board, List<Mat> detectedCorners, Mat detectedIds, List<Mat> rejectedCorners)
    {
        Mat detectedCorners_mat = Converters.vector_Mat_to_Mat(detectedCorners);
        Mat rejectedCorners_mat = Converters.vector_Mat_to_Mat(rejectedCorners);
        refineDetectedMarkers_1(image.nativeObj, board.getNativeObjAddr(), detectedCorners_mat.nativeObj, detectedIds.nativeObj, rejectedCorners_mat.nativeObj);
        Converters.Mat_to_vector_Mat(detectedCorners_mat, detectedCorners);
        detectedCorners_mat.release();
        Converters.Mat_to_vector_Mat(rejectedCorners_mat, rejectedCorners);
        rejectedCorners_mat.release();
        return;
    }




    // C++:  Ptr_Dictionary generateCustomDictionary(int nMarkers, int markerSize, Ptr_Dictionary baseDictionary)
    private static native long custom_dictionary_from_0(int nMarkers, int markerSize, long baseDictionary_nativeObj);

    // C++:  Ptr_Dictionary generateCustomDictionary(int nMarkers, int markerSize)
    private static native long custom_dictionary_0(int nMarkers, int markerSize);

    // C++:  Ptr_Dictionary getPredefinedDictionary(int dict)
    private static native long getPredefinedDictionary_0(int dict);

    // C++:  bool estimatePoseCharucoBoard(Mat charucoCorners, Mat charucoIds, Ptr_CharucoBoard board, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false)
    private static native boolean estimatePoseCharucoBoard_0(long charucoCorners_nativeObj, long charucoIds_nativeObj, long board_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess);
    private static native boolean estimatePoseCharucoBoard_1(long charucoCorners_nativeObj, long charucoIds_nativeObj, long board_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  double calibrateCameraAruco(vector_Mat corners, Mat ids, Mat counter, Ptr_Board board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    private static native double calibrateCameraArucoExtended_0(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraArucoExtended_1(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double calibrateCameraArucoExtended_2(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj);

    // C++:  double calibrateCameraAruco(vector_Mat corners, Mat ids, Mat counter, Ptr_Board board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs = vector_Mat(), vector_Mat& tvecs = vector_Mat(), int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    private static native double calibrateCameraAruco_0(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraAruco_1(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);
    private static native double calibrateCameraAruco_2(long corners_mat_nativeObj, long ids_nativeObj, long counter_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);

    // C++:  double calibrateCameraCharuco(vector_Mat charucoCorners, vector_Mat charucoIds, Ptr_CharucoBoard board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs, vector_Mat& tvecs, Mat& stdDeviationsIntrinsics, Mat& stdDeviationsExtrinsics, Mat& perViewErrors, int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    private static native double calibrateCameraCharucoExtended_0(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraCharucoExtended_1(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj, int flags);
    private static native double calibrateCameraCharucoExtended_2(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, long stdDeviationsIntrinsics_nativeObj, long stdDeviationsExtrinsics_nativeObj, long perViewErrors_nativeObj);

    // C++:  double calibrateCameraCharuco(vector_Mat charucoCorners, vector_Mat charucoIds, Ptr_CharucoBoard board, Size imageSize, Mat& cameraMatrix, Mat& distCoeffs, vector_Mat& rvecs = vector_Mat(), vector_Mat& tvecs = vector_Mat(), int flags = 0, TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON))
    private static native double calibrateCameraCharuco_0(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags, int criteria_type, int criteria_maxCount, double criteria_epsilon);
    private static native double calibrateCameraCharuco_1(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_mat_nativeObj, long tvecs_mat_nativeObj, int flags);
    private static native double calibrateCameraCharuco_2(long charucoCorners_mat_nativeObj, long charucoIds_mat_nativeObj, long board_nativeObj, double imageSize_width, double imageSize_height, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);

    // C++:  int estimatePoseBoard(vector_Mat corners, Mat ids, Ptr_Board board, Mat cameraMatrix, Mat distCoeffs, Mat& rvec, Mat& tvec, bool useExtrinsicGuess = false)
    private static native int estimatePoseBoard_0(long corners_mat_nativeObj, long ids_nativeObj, long board_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, boolean useExtrinsicGuess);
    private static native int estimatePoseBoard_1(long corners_mat_nativeObj, long ids_nativeObj, long board_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj);

    // C++:  int interpolateCornersCharuco(vector_Mat markerCorners, Mat markerIds, Mat image, Ptr_CharucoBoard board, Mat& charucoCorners, Mat& charucoIds, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat(), int minMarkers = 2)
    private static native int interpolateCornersCharuco_0(long markerCorners_mat_nativeObj, long markerIds_nativeObj, long image_nativeObj, long board_nativeObj, long charucoCorners_nativeObj, long charucoIds_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, int minMarkers);
    private static native int interpolateCornersCharuco_1(long markerCorners_mat_nativeObj, long markerIds_nativeObj, long image_nativeObj, long board_nativeObj, long charucoCorners_nativeObj, long charucoIds_nativeObj);

    // C++:  void detectCharucoDiamond(Mat image, vector_Mat markerCorners, Mat markerIds, float squareMarkerLengthRate, vector_Mat& diamondCorners, Mat& diamondIds, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat())
    private static native void detectCharucoDiamond_0(long image_nativeObj, long markerCorners_mat_nativeObj, long markerIds_nativeObj, float squareMarkerLengthRate, long diamondCorners_mat_nativeObj, long diamondIds_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj);
    private static native void detectCharucoDiamond_1(long image_nativeObj, long markerCorners_mat_nativeObj, long markerIds_nativeObj, float squareMarkerLengthRate, long diamondCorners_mat_nativeObj, long diamondIds_nativeObj);

    // C++:  void detectMarkers(Mat image, Ptr_Dictionary dictionary, vector_Mat& corners, Mat& ids, Ptr_DetectorParameters parameters = DetectorParameters::create(), vector_Mat& rejectedImgPoints = vector_Mat(), Mat cameraMatrix = Mat(), Mat distCoeff = Mat())
    private static native void detectMarkers_0(long image_nativeObj, long dictionary_nativeObj, long corners_mat_nativeObj, long ids_nativeObj, long parameters_nativeObj, long rejectedImgPoints_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeff_nativeObj);
    private static native void detectMarkers_1(long image_nativeObj, long dictionary_nativeObj, long corners_mat_nativeObj, long ids_nativeObj);

    // C++:  void drawAxis(Mat& image, Mat cameraMatrix, Mat distCoeffs, Mat rvec, Mat tvec, float length)
    private static native void drawAxis_0(long image_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvec_nativeObj, long tvec_nativeObj, float length);

    // C++:  void drawDetectedCornersCharuco(Mat& image, Mat charucoCorners, Mat charucoIds = Mat(), Scalar cornerColor = Scalar(255, 0, 0))
    private static native void drawDetectedCornersCharuco_0(long image_nativeObj, long charucoCorners_nativeObj, long charucoIds_nativeObj, double cornerColor_val0, double cornerColor_val1, double cornerColor_val2, double cornerColor_val3);
    private static native void drawDetectedCornersCharuco_1(long image_nativeObj, long charucoCorners_nativeObj);

    // C++:  void drawDetectedDiamonds(Mat& image, vector_Mat diamondCorners, Mat diamondIds = Mat(), Scalar borderColor = Scalar(0, 0, 255))
    private static native void drawDetectedDiamonds_0(long image_nativeObj, long diamondCorners_mat_nativeObj, long diamondIds_nativeObj, double borderColor_val0, double borderColor_val1, double borderColor_val2, double borderColor_val3);
    private static native void drawDetectedDiamonds_1(long image_nativeObj, long diamondCorners_mat_nativeObj);

    // C++:  void drawDetectedMarkers(Mat& image, vector_Mat corners, Mat ids = Mat(), Scalar borderColor = Scalar(0, 255, 0))
    private static native void drawDetectedMarkers_0(long image_nativeObj, long corners_mat_nativeObj, long ids_nativeObj, double borderColor_val0, double borderColor_val1, double borderColor_val2, double borderColor_val3);
    private static native void drawDetectedMarkers_1(long image_nativeObj, long corners_mat_nativeObj);

    // C++:  void drawMarker(Ptr_Dictionary dictionary, int id, int sidePixels, Mat& img, int borderBits = 1)
    private static native void drawMarker_0(long dictionary_nativeObj, int id, int sidePixels, long img_nativeObj, int borderBits);
    private static native void drawMarker_1(long dictionary_nativeObj, int id, int sidePixels, long img_nativeObj);

    // C++:  void drawPlanarBoard(Ptr_Board board, Size outSize, Mat& img, int marginSize = 0, int borderBits = 1)
    private static native void drawPlanarBoard_0(long board_nativeObj, double outSize_width, double outSize_height, long img_nativeObj, int marginSize, int borderBits);
    private static native void drawPlanarBoard_1(long board_nativeObj, double outSize_width, double outSize_height, long img_nativeObj);

    // C++:  void estimatePoseSingleMarkers(vector_Mat corners, float markerLength, Mat cameraMatrix, Mat distCoeffs, Mat& rvecs, Mat& tvecs, Mat& _objPoints = Mat())
    private static native void estimatePoseSingleMarkers_0(long corners_mat_nativeObj, float markerLength, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_nativeObj, long tvecs_nativeObj, long _objPoints_nativeObj);
    private static native void estimatePoseSingleMarkers_1(long corners_mat_nativeObj, float markerLength, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, long rvecs_nativeObj, long tvecs_nativeObj);

    // C++:  void getBoardObjectAndImagePoints(Ptr_Board board, vector_Mat detectedCorners, Mat detectedIds, Mat& objPoints, Mat& imgPoints)
    private static native void getBoardObjectAndImagePoints_0(long board_nativeObj, long detectedCorners_mat_nativeObj, long detectedIds_nativeObj, long objPoints_nativeObj, long imgPoints_nativeObj);

    // C++:  void refineDetectedMarkers(Mat image, Ptr_Board board, vector_Mat& detectedCorners, Mat& detectedIds, vector_Mat& rejectedCorners, Mat cameraMatrix = Mat(), Mat distCoeffs = Mat(), float minRepDistance = 10.f, float errorCorrectionRate = 3.f, bool checkAllOrders = true, Mat& recoveredIdxs = Mat(), Ptr_DetectorParameters parameters = DetectorParameters::create())
    private static native void refineDetectedMarkers_0(long image_nativeObj, long board_nativeObj, long detectedCorners_mat_nativeObj, long detectedIds_nativeObj, long rejectedCorners_mat_nativeObj, long cameraMatrix_nativeObj, long distCoeffs_nativeObj, float minRepDistance, float errorCorrectionRate, boolean checkAllOrders, long recoveredIdxs_nativeObj, long parameters_nativeObj);
    private static native void refineDetectedMarkers_1(long image_nativeObj, long board_nativeObj, long detectedCorners_mat_nativeObj, long detectedIds_nativeObj, long rejectedCorners_mat_nativeObj);

}
