package org.firstinspires.ftc.teamcode.hardware;

import org.opencv.core.Mat;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;


public class ConeQRProcessor  extends OpenCvPipeline {

    QRCodeDetector decoder = new QRCodeDetector();
    String qrCode;
    SleeveSide sideDetected = SleeveSide.Unkown;

    boolean debug = true;

    private StringBuffer detectMsgBuf;
    private String detectMsg  ;
    boolean decoded = false;


    @Override
    public Mat processFrame (Mat input) {
        long startMills  = System.currentTimeMillis();
        detectMsgBuf = new StringBuffer();

        Mat result = input;
        Mat points = new Mat();

        qrCode = decoder.detectAndDecodeCurved(input,points);
        if ( qrCode!=null) {
            decoded = true;
            if (qrCode.equals("S1")) {
                this.sideDetected = SleeveSide.Sleev1;
            } else if (this.qrCode.equals("S2")) {
                this.sideDetected = SleeveSide.Sleev2;
            } else if (this.qrCode.equals("S3")) {
                this.sideDetected = SleeveSide.Sleev3;
            }
        }

        if (debug) {
            File imgFile = new File("/sdcard/FIRST/Cone-result.jpg");
            if ( !imgFile.exists()) {
                Imgcodecs.imwrite("/sdcard/FIRST/Cone-result.jpg", input);
            }
        }

        detectMsgBuf.append(qrCode);

        long endMills  = System.currentTimeMillis();
        detectMsgBuf.append(" Duration: " + (endMills - startMills) );
        detectMsg = detectMsgBuf.toString();

        return result;
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
