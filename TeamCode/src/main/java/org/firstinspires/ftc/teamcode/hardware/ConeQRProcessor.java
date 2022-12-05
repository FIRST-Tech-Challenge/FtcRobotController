package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Core;
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
                Mat result = proc.processFrame(test01);
                System.out.println("File: " + files[i].getAbsolutePath());
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
        Mat brReverse = input;

        //Imgproc.cvtColor(input, brReverse, Imgproc.COLOR_BGR2RGB);
        //Imgproc.resize(input,brReverse, new Size(540,960 ));

        long startMills = 0;
        long endMills = 0;
        List<String> codes = new ArrayList<String> () ;
        Mat points  = new Mat();

        if ( !decoded) {
            startMills  = System.currentTimeMillis();
            qrCode = decoder.detectAndDecodeCurved(brReverse, points);
            endMills  = System.currentTimeMillis();
            System.out.println(points.dump());
            if (!points.empty()) {
                decoded = true;
                System.out.println("QR code: " + qrCode);
                for (int i = 0; i < points.cols(); i++) {
                    Point pt1 = new Point(points.get(0, i));
                    Point pt2 = new Point(points.get(0, (i + 1) % 4));
                    Imgproc.line(brReverse, pt1, pt2, new Scalar(255, 0, 0), 3);
                }
            }
            triedTimes++;
        }

        if (decoded) {
            System.out.println("QR code:" + qrCode);

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
            File imgFile = new File("/sdcard/FIRST/qr-result.jpg");
            if ( !imgFile.exists()) {
                Imgcodecs.imwrite("/sdcard/FIRST/qr-result.jpg", brReverse);
            }
        }

        detectMsgBuf.append("qr code: " + qrCode + " Tried: " + triedTimes);
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

}
