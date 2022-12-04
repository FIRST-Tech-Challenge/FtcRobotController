package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Mat;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;

/**
 * This is the class implement OpenCVPipleline that detect QR code.
 *
 */
public class ConeQRProcessor  extends OpenCvPipeline {

    QRCodeDetector decoder = new QRCodeDetector();
    String qrCode;
    SleeveSide sideDetected = SleeveSide.Unkown;

    boolean debug = true;

    private StringBuffer detectMsgBuf;
    private String detectMsg  ;
    boolean decoded = false;

    int triedTimes = 0;

    @Override
    public Mat processFrame (Mat input) {

        detectMsgBuf = new StringBuffer();
        Mat brReverse = new Mat() ;

        Imgproc.cvtColor(input, brReverse, Imgproc.COLOR_BGR2RGB);

        long startMills = 0;
        long endMills = 0;

        if ( !decoded) {
            startMills  = System.currentTimeMillis();
            qrCode = decoder.detectAndDecode(brReverse);
            endMills  = System.currentTimeMillis();
            triedTimes++;
        }

        if ( qrCode!=null && !qrCode.equals("") && !decoded) {
            System.out.println("QR code:" + qrCode);
            decoded = true;
            if (qrCode.equals("1")) {
                this.sideDetected = SleeveSide.Sleev1;
            } else if (this.qrCode.equals("2")) {
                this.sideDetected = SleeveSide.Sleev2;
            } else if (this.qrCode.equals("3")) {
                this.sideDetected = SleeveSide.Sleev3;
            }
        }

        if (debug) {
            File imgFile = new File("/sdcard/FIRST/Cone-result.jpg");
            if ( !imgFile.exists()) {
                Imgcodecs.imwrite("/sdcard/FIRST/Cone-result.jpg", brReverse);
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
