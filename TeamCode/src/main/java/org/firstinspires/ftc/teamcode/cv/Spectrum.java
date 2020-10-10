//package org.firstinspires.ftc.teamcode.cv;
//
//import org.opencv.core.*;
//import org.opencv.highgui.HighGui;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//
//public class Spectrum {
//    public static void main(String[] args) {
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//
//        Mat s = new Mat(100, 181*3, CvType.CV_8UC3);
//        for(int i=0;i<180;i++){
//            Imgproc.rectangle(s, new Point(i*3,0),new Point((i+1)*3, 50), new Scalar(i, 255, 255), -1);
//
//            if(i%10 == 0)
//                Imgproc.putText(s, String.valueOf(i), new Point(i*3, 70), Imgproc.FONT_HERSHEY_PLAIN, 1, new Scalar(0,0,255));
//        }
//
//        Imgproc.cvtColor(s,s,Imgproc.COLOR_HSV2BGR);
//
//        HighGui.imshow("kkk", s);
//        HighGui.waitKey();
//        System.exit(0);
//    }
//
//
//}
