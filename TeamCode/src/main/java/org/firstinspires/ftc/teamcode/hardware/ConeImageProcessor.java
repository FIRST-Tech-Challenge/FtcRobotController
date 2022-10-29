package org.firstinspires.ftc.teamcode.hardware;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class ConeImageProcessor {

	boolean debug = true;

	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		Mat test01 = Imgcodecs.imread("D:\\eclipse-workspace\\Testing01.jpg");

		ConeImageProcessor proc = new ConeImageProcessor();
		proc.processFrame2(test01);

		// System.out.println("mat = " + test01.dump());

	}


	public Mat processFrame2 (Mat input) {
		//Convert to HSV color space. 		
		Mat hsvImg = new Mat();
		hsvImg.create(input.size(), CvType.CV_8U);
		Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);

		//Find Blue color in range. 
		Mat thresh = new Mat();		
		Core.inRange(hsvImg, new Scalar(100, 100, 20 ), new Scalar(130,255,255), thresh);
		
		///Apply mask to input image, and copy to new image. 
		Mat result  = new Mat(input.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
		input.copyTo(result, thresh);
		
		List<Mat> hsvThreholdPlane = new ArrayList<>();
		Core.split(result, hsvThreholdPlane);
		
		if (debug) {
			Imgcodecs.imwrite("D:\\eclipse-workspace\\thresh.jpg", thresh);
			Imgcodecs.imwrite("D:\\eclipse-workspace\\result.jpg", result);
			Imgcodecs.imwrite("D:\\eclipse-workspace\\foreGroundH.jpg", hsvThreholdPlane.get(0));
			System.out.println("Result = " + hsvThreholdPlane.get(0).dump());
		}

		return result;
	}

	
	
	/**
	 * This is used by pipeline
	 * 
	 * @param input
	 * @return
	 */
	public Mat processFrame(Mat input) {
		Mat hsvImg = new Mat();
		hsvImg.create(input.size(), CvType.CV_8U);
		Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);

		// Now split Hue, Saturation, and Value.
		List<Mat> hsvPlanes = new ArrayList<>();
		Core.split(hsvImg, hsvPlanes);

		//Threshold by Saturation 
		Mat sThresholdImg = new Mat();
		int staturationThreshold = 25;
		Imgproc.threshold(hsvPlanes.get(1), sThresholdImg, staturationThreshold, 255, Imgproc.THRESH_BINARY);
		
		//Threshold by Hue 
		Mat hThresholdImg = new Mat();
		int hueThreshold = 40;
		Imgproc.threshold(hsvPlanes.get(0), hThresholdImg, hueThreshold, 180, Imgproc.THRESH_BINARY);

		//combine 2 mask 
		Mat thresholdImg = new Mat(); 
		Core.bitwise_and(sThresholdImg, sThresholdImg, thresholdImg); 	

		
		// dilate to fill gaps, erode to smooth edges
		Imgproc.dilate(thresholdImg, thresholdImg, new Mat(), new Point(-1, -1), 1);
		Imgproc.erode(thresholdImg, thresholdImg, new Mat(), new Point(-1, -1), 3);
		
		//Imgproc.threshold(thresholdImg, thresholdImg, staturationThreshold, 255, Imgproc.THRESH_BINARY);		
		
		Mat foreground = new Mat(input.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
		input.copyTo(foreground, thresholdImg);
		
		//Calculate Hue average
		List<Mat> hsvThreholdPlane = new ArrayList<>();
		Core.split(foreground, hsvThreholdPlane);
		
		if (debug) {
			Imgcodecs.imwrite("D:\\eclipse-workspace\\H.jpg", hsvPlanes.get(0));
			Imgcodecs.imwrite("D:\\eclipse-workspace\\S.jpg", hsvPlanes.get(1));
			//Imgcodecs.imwrite("D:\\eclipse-workspace\\V.jpg", hsvPlanes.get(2));
			Imgcodecs.imwrite("D:\\eclipse-workspace\\threshold.jpg", thresholdImg);
			Imgcodecs.imwrite("D:\\eclipse-workspace\\thresholdh.jpg", hThresholdImg);
			Imgcodecs.imwrite("D:\\eclipse-workspace\\thresholds.jpg", sThresholdImg);
			Imgcodecs.imwrite("D:\\eclipse-workspace\\foreGround.jpg", foreground);
			System.out.println("Result = " + hsvThreholdPlane.get(0).dump());
		}

		return foreground;
	}

	
}
