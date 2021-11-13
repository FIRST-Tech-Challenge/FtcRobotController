package org.firstinspires.ftc.teamcode.samples.FaceDetector;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfRect;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.CascadeClassifier;
import org.opencv.objdetect.Objdetect;
import org.openftc.easyopencv.OpenCvPipeline;
import android.content.res.AssetManager;
import android.content.res.Resources;

import java.util.ArrayList;
import java.util.List;

public class FaceDetector extends OpenCvPipeline {

    private int width; // width of the image


    /**
     *
     * @param width The width of the image (check your camera)
     */
    public FaceDetector(int width) {
        this.width = width;
    }

    //AssetManager assetManager = AssetManager.AssetInputStream();


    @Override
    public Mat processFrame(Mat inputImage) {

        MatOfRect facesDetected = new MatOfRect();
        CascadeClassifier cascadeClassifier = new CascadeClassifier();

        int minFaceSize = Math.round(inputImage.rows() * 0.1f);
        cascadeClassifier.load("/sdcard/FIRST/opencv/models/haarcascade_frontalface_alt.xml");
        cascadeClassifier.detectMultiScale(inputImage,
                facesDetected,
                1.1,
                3,
                Objdetect.CASCADE_SCALE_IMAGE,
                new Size(minFaceSize, minFaceSize),
                new Size()
        );
        Rect[] facesArray =  facesDetected.toArray();
        for(Rect face : facesArray) {
            Imgproc.rectangle(inputImage, face.tl(), face.br(), new Scalar(0, 0, 255), 3 );
        }
        return inputImage;
    }

}