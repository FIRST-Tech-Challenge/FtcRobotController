package org.firstinspires.ftc.teamcode.hardware;

import android.util.Log;

import  org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

/**
 * This is the class implement OpenCVPipleline that detect QR code.
 *
 */
public class ConeQRProcessor  extends OpenCvPipeline {

    double meanVal =0.0;

    //Adjustable parameters here.
    double sleev1Peak = 50;
    double sleev2Peak = 100.0 ;
    double sleev3Peak = 150.0;

    //Range for background, mat is grey
    Scalar backgroundL = new Scalar(0, 0, 0 );
    Scalar backgroundH = new Scalar(255, 150, 255 );

    //Range for too dark part of pic
    Scalar darkL = new Scalar(0, 0, 0 );
    Scalar darkH = new Scalar(179, 255, 50 );

    QRCodeDetector decoder = new QRCodeDetector();
    SleeveSide sideDetected = SleeveSide.Unkown;

    boolean debug = true;

    private StringBuffer detectMsgBuf;
    private String detectMsg  ;
    boolean decoded = false;

    int triedTimes = 0;

    public static void main(String[] args) {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        ConeQRProcessor proc = new ConeQRProcessor();

        File imageDir = new File("C:\\FTC Code\\TestPIcs");
        File resultDir = new File("C:\\FTC Code\\TestPIcs\\result");

        File[] files = imageDir.listFiles();
        for (int i = 0; i < files.length; i++) {
            if (files[i].isFile()) {
                Mat test01 = Imgcodecs.imread(files[i].getAbsolutePath());
                Mat bgr = new Mat();
                proc.triedTimes = 0;
                proc.decoded = false;
                System.out.println("File: " + files[i].getAbsolutePath());
                //After Read file, convert it to BGR color space, before send to processing.
                //Imgproc.cvtColor(test01, bgr, Imgproc.COLOR_RGB2BGR);
                Mat result = proc.processFrame(test01);
                System.out.println("Message: " + proc.getDetectMsg());
                System.out.println("Detected: " + proc.isDecoded() + "\n");
                //Imgcodecs.imwrite(resultDir.getAbsolutePath() + "\\" + files[i].getName() + "-result.jpg", result);

            }
        }

    }

    public void setDecoded(boolean decoded) {
        this.decoded = decoded;
    }

    @Override
    public Mat processFrame (Mat input) {
        Log.d("9010", "Entering processFrame isDecoded: "  + isDecoded());
        String qrCode = null;

        detectMsgBuf = new StringBuffer();

        long startMills = 0;
        long endMills = 0;
        List<String> codes = new ArrayList<String> () ;
        Mat points  = new Mat();

        Mat croppedQR = null;

        if ( !decoded) {
            Log.d("9010", "Starting qr code process ");
            startMills  = System.currentTimeMillis();
            Mat grey = new Mat();
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGB2GRAY);
            qrCode = decoder.detectAndDecodeCurved(grey, points);
            endMills  = System.currentTimeMillis();
            Log.d("9010", "qr code coordination: " + points.dump() + " QR code: " + qrCode);
            if (!points.empty()) {
                //Detected. However not necessary decode correctly.

                Log.d("9010", "QR code by decoder: " + qrCode);

                /*
                for (int i = 0; i < points.cols(); i++) {
                    Point pt1 = new Point(points.get(0, i));
                    Point pt2 = new Point(points.get(0, (i + 1) % 4));
                    Imgproc.line(input, pt1, pt2, new Scalar(255, 0, 0), 1);
                }*/

                if ( qrCode == null || "".equals(qrCode)) {
                    //If not decoded correctly, tried to get color to find out qrCode.
                    detectMsgBuf.append(" Decode Fail, using color ");
                    Log.d("9010", " Decode Fail, using color ");

                    //Crop image from point 0 and point 3
                    Rect range = new Rect(new Point(points.get(0, 1)),  new Point(points.get(0, 3))  );
                    croppedQR = new Mat ( input, range);

                    //get HSV
                    Mat hsvImg = new Mat();
                    hsvImg.create(croppedQR.size(), CvType.CV_8U);
                    Imgproc.cvtColor( croppedQR, hsvImg, Imgproc.COLOR_RGB2HSV);

                    //Remove black part.
                    Mat darkTrd = new Mat();
                    Core.inRange(hsvImg, darkL, darkH, darkTrd);
                    //Imgcodecs.imwrite("c:/sdcard/FIRST/background.jpg",darkTrd );

                    //Get reversed Mask
                    Mat tobeRemove = new Mat();
                    Core.bitwise_not(darkTrd,tobeRemove);

                    //Apply mask to input image, and copy to new image.
                    Mat result  = new Mat(croppedQR.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
                    croppedQR.copyTo(result, tobeRemove);
                    //Imgcodecs.imwrite("c:/sdcard/FIRST/crop-removed.jpg",result );

                    Mat hsvResult = new Mat();
                    Imgproc.cvtColor(result, hsvResult, Imgproc.COLOR_RGB2HSV);

                    //Calculate the Hue average
                    meanVal = calHueAvg(hsvResult);
                    System.out.println("Mean Value: " +  meanVal);
                    //System.out.println( hsvResult.dump() );

                    //Calculate the diff to peak
                    double diff1 = Math.abs(meanVal - sleev1Peak);
                    double  diff2 = Math.abs(meanVal - sleev2Peak);
                    double diff3 = Math.abs(meanVal - sleev3Peak);

                    double smallest = Math.min(diff1, Math.min(diff2, diff3));
                    if ( smallest == diff1) {
                        qrCode = "S1";
                    } else if (smallest == diff2 ){
                        qrCode = "S2";

                    } else if ( smallest == diff3 ) {
                        qrCode = "S3";
                    }

                }

                decoded = true;
            }
            triedTimes++;
        }

        if (decoded && qrCode !=null ) {
            if (qrCode.equals("S1")) {
                this.sideDetected = SleeveSide.Sleev1;
            } else if (qrCode.equals("S2")) {
                this.sideDetected = SleeveSide.Sleev2;
            } else if (qrCode.equals("S3")) {
                this.sideDetected = SleeveSide.Sleev3;
            } else  {
                this.sideDetected = SleeveSide.Unkown;
            }
        }

        if (debug) {
            Imgcodecs.imwrite("/sdcard/FIRST/qr-result.jpg", input);

            if ( croppedQR !=null ) {
                Imgcodecs.imwrite("/sdcard/FIRST/qr-cropeed.jpg", croppedQR);
            }

        }

        detectMsgBuf.append("qr code: " + qrCode + " Tried: " + triedTimes + " mean value: " + meanVal);
        detectMsgBuf.append(" Duration: " + (endMills - startMills) );
        detectMsg = detectMsgBuf.toString();
        Log.d("9010", detectMsg);

        return input;
    }


    public SleeveSide getSleeveSide ( ) {

        return sideDetected;
    }

    public String getDetectMsg() {
        return detectMsg;
    }

    public boolean isDecoded() {
        return decoded;
    }


    private double  calHueAvg ( Mat input ) {
        double  total = 0.0;
        int count = 0;

        for ( int h = 0; h < input.size().height; h ++ ) {
            for ( int w = 0; w < input.size().width; w ++ ) {
                byte[]  data = {0,0,0} ;
                input.get(h,w,data);

                int hue = Byte.toUnsignedInt( data[0]);

                if ( debug) {
                    //System.out.println("data is: " + hue );
                }
                if ( hue> 0 && hue < 180 ) {
                    //System.out.println("hue : " + hue + " Posiiton: " + h + "," + w  );
                    total += hue;
                    count ++;
                }

            }
        }

        if ( debug) {
            //System.out.println("Count is: " + count );

        }
        return total/count;

    }

}
