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
	//Blue Range for the blue cone
	Scalar blueConeL = new Scalar(105, 60, 20 );
	Scalar blueConeH = new Scalar(115, 255, 255 );

	//Red Range for red cone
	Scalar redConeL1 = new Scalar(0, 60, 20 );
	Scalar redConeH1 = new Scalar(15, 255, 255 );
	Scalar redConeL2 = new Scalar(170, 60, 20 );
	Scalar redConeH2 = new Scalar(179, 255, 255 );

	//Range for background, mat is grey
	Scalar backgroundL = new Scalar(0, 0, 20 );
	Scalar backgroundH = new Scalar(179, 27, 255 );

    //Range for yellow pol
	Scalar yellowPoleL = new Scalar(15, 60, 20 );
	Scalar yellowPoleH = new Scalar(25, 255, 255 );


	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

		Mat test01 = Imgcodecs.imread("C:\\FTC Code\\TestPIcs\\20221029_095209.jpg");

		ConeImageProcessor proc = new ConeImageProcessor();
		proc.processFrame(test01);

		// System.out.println("mat = " + test01.dump());

	}


	public Mat processFrame (Mat input) {
		//Convert to HSV color space. 		
		Mat hsvImg = new Mat();
		hsvImg.create(input.size(), CvType.CV_8U);
		Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);

		//Find Blue cone in range.
		Mat blueConeTrd = new Mat();
		Core.inRange(hsvImg, blueConeL, blueConeH, blueConeTrd);

		Mat redConeTrd1 = new Mat();
		Core.inRange(hsvImg, redConeL1, redConeH1, redConeTrd1);
		Mat redConeTrd2 = new Mat();
		Core.inRange(hsvImg, redConeL2, redConeH2, redConeTrd2);

		Mat yellowPolTrd = new Mat ();
		Core.inRange(hsvImg, yellowPoleL, yellowPoleH, yellowPolTrd);

		Mat backroundTrd = new Mat();
		Core.inRange(hsvImg, backgroundL, backgroundH, backroundTrd);


		Mat tobeRemoveRev = new Mat();
		Core.bitwise_or(blueConeTrd,redConeTrd1,tobeRemoveRev);
		Core.bitwise_or(redConeTrd2,tobeRemoveRev,tobeRemoveRev);
		Core.bitwise_or(yellowPolTrd,tobeRemoveRev,tobeRemoveRev);
		Core.bitwise_or(backroundTrd,tobeRemoveRev,tobeRemoveRev);

		Mat tobeRemove = new Mat();
		Core.bitwise_not(tobeRemoveRev,tobeRemove);

		///Apply mask to input image, and copy to new image. 
		Mat result  = new Mat(input.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
		input.copyTo(result, tobeRemove);
		
		List<Mat> hsvThreholdPlane = new ArrayList<>();
		Core.split(result, hsvThreholdPlane);
		
		if (debug) {
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\thresh.jpg", tobeRemoveRev);
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\result.jpg", result);
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\foreGroundH.jpg", hsvThreholdPlane.get(0));
			System.out.println("Result = " + hsvThreholdPlane.get(0).dump());
		}

		return result;
	}

	
	
	/**
	 * This is used by pipeline
	 * 
	 * @param input
	 * @return

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
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\H.jpg", hsvPlanes.get(0));
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\S.jpg", hsvPlanes.get(1));
			//Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\V.jpg", hsvPlanes.get(2));
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\threshold.jpg", thresholdImg);
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\thresholdh.jpg", hThresholdImg);
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\thresholds.jpg", sThresholdImg);
			Imgcodecs.imwrite("C:\\FTC Code\\TestPIcs\\foreGround.jpg", foreground);
			System.out.println("Result = " + hsvThreholdPlane.get(0).dump());
		}

		return foreground;
	}

	 */
}
