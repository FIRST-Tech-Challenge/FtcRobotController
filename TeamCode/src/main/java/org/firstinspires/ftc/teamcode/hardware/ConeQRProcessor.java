package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Core;
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
    double sleev1Peak = 30;
    double sleev2Peak = 100.0 ;
    double sleev3Peak = 150.0;

    //Range for background, mat is grey
    Scalar backgroundL = new Scalar(0, 0, 0 );
    Scalar backgroundH = new Scalar(255, 120, 255 );

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
                proc.triedTimes = 0;
                proc.decoded = false;
                System.out.println("File: " + files[i].getAbsolutePath());
                Mat result = proc.processFrame(test01);
                System.out.println("Message: " + proc.getDetectMsg());
                System.out.println("Detected: " + proc.isDecoded() + "\n");
                //Imgcodecs.imwrite(resultDir.getAbsolutePath() + "\\" + files[i].getName() + "-result.jpg", result);

            }
        }

    }

    @Override
    public Mat processFrame (Mat input) {
        String qrCode = null;

        detectMsgBuf = new StringBuffer();
        Mat brReverse = new Mat();

        //This is not needed for use with Carema.  If loading image from Disk not needed.
        Imgproc.cvtColor(input, brReverse, Imgproc.COLOR_BGR2RGB);

        long startMills = 0;
        long endMills = 0;
        List<String> codes = new ArrayList<String> () ;
        Mat points  = new Mat();

        Mat croppedQR = null;

        if ( !decoded) {
            startMills  = System.currentTimeMillis();
            Mat grey = new Mat();
            Imgproc.cvtColor(brReverse, grey, Imgproc.COLOR_BGR2GRAY);
            qrCode = decoder.detectAndDecodeCurved(grey, points);
            endMills  = System.currentTimeMillis();
            System.out.println(points.dump());
            if (!points.empty()) {
                //Detected. However not necessary decode correctly.
                System.out.println("QR code: " + qrCode);

                for (int i = 0; i < points.cols(); i++) {
                    Point pt1 = new Point(points.get(0, i));
                    Point pt2 = new Point(points.get(0, (i + 1) % 4));
                    Imgproc.line(brReverse, pt1, pt2, new Scalar(255, 0, 0), 1);
                }

                if ( qrCode == null || "".equals(qrCode)) {
                    //If not decoded correctly, tried to get color to find out qrCode.
                    detectMsgBuf.append(" Decode Fail, using color ");

                    //Crop image from point 0 and point 3
                    Rect range = new Rect(new Point(points.get(0, 1)),  new Point(points.get(0, 3))  );
                    croppedQR = new Mat ( brReverse, range);

                    //get HSV
                    Mat hsvImg = new Mat();
                    hsvImg.create(croppedQR.size(), CvType.CV_8U);
                    Imgproc.cvtColor( croppedQR, hsvImg, Imgproc.COLOR_BGR2HSV);

                    //Remove background with low saturation
                    Mat backgroundTrd = new Mat();
                    Core.inRange(hsvImg, backgroundL, backgroundH, backgroundTrd);

                    //Get reversed Mask
                    Mat tobeRemove = new Mat();
                    Core.bitwise_not(backgroundTrd,tobeRemove);

                    //Apply mask to input image, and copy to new image.
                    Mat result  = new Mat(croppedQR.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
                    croppedQR.copyTo(result, tobeRemove);

                    Mat hsvResult = new Mat();
                    Imgproc.cvtColor(result, hsvResult, Imgproc.COLOR_BGR2HSV);

                    //Calculate the Hue average
                    meanVal = calHueAvg(hsvResult);
                    System.out.println("Mean Value: " +  meanVal);
                    //System.out.println( hsvResult.dump() );

                    //Calculate the diff to peak
                    double diff1 = Math.abs(meanVal - sleev1Peak);
                    double diff2 = Math.abs(meanVal - sleev2Peak);
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

            Imgcodecs.imwrite("/sdcard/FIRST/qr-result.jpg", brReverse);
            if ( croppedQR !=null ) {
                Imgcodecs.imwrite("/sdcard/FIRST/qr-cropeed.jpg", croppedQR);
            }


        }

        detectMsgBuf.append("qr code: " + qrCode + " Tried: " + triedTimes + " mean value: " + meanVal);
        detectMsgBuf.append(" Duration: " + (endMills - startMills) );
        detectMsg = detectMsgBuf.toString();

        return brReverse;
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
