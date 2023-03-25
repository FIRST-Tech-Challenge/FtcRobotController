package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.BarcodeDeterminationBlue;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

public class ConeImageProcessor extends OpenCvPipeline {

	//Adjustable parameters here.
	double sleev1Peak = 25;
	double sleev2Peak = 100.0;
	double sleev3Peak = 150;

	File fileProcessing;

	boolean debug = true;
	double  meanVal = 0.0;

	//Blue Range for the blue cone
	Scalar blueConeL = new Scalar(105, 100, 00 );
	Scalar blueConeH = new Scalar(119, 255, 255 );

	//Red Range for red cone
	Scalar redConeL1 = new Scalar(0, 100, 00 );
	Scalar redConeH1 = new Scalar(15, 255, 255 );
	Scalar redConeL2 = new Scalar(165, 100, 00 );
	Scalar redConeH2 = new Scalar(179, 255, 255 );

	//Range for background, mat is grey
	Scalar backgroundL = new Scalar(0, 0, 0 );
	Scalar backgroundH = new Scalar(255, 120, 255 );

    //Range for yellow pol
	Scalar yellowPoleL = new Scalar(15, 100, 0 );
	Scalar yellowPoleH = new Scalar(25, 255, 255 );

	//Range for too dark part of pic
	Scalar darkL = new Scalar(0, 0, 0 );
	Scalar darkH = new Scalar(179, 255, 50 );

	public File getFileProcessing() {
		return fileProcessing;
	}

	public void setFileProcessing(File fileProcessing) {
		this.fileProcessing = fileProcessing;
	}

	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		ConeImageProcessor proc = new ConeImageProcessor();

		File imageDir = new File("C:\\FTC Code\\TestPIcs" );
		File resultDir = new File("C:\\FTC Code\\TestPIcs\\result" );

		File[] files = imageDir.listFiles();
		for ( int i =0; i < files.length; i++) {
			if ( files[i].isFile()) {
				Mat test01 = Imgcodecs.imread(files[i].getAbsolutePath());
				proc.setFileProcessing(files[i]);
				proc.processFrame(test01);
				System.out.println("File: " + files[i].getName() + " value:" + proc.meanVal);
			}
		}
	}

	public Double getMeanVal() {
		return meanVal;
	}

	@Override
	public Mat processFrame (Mat input) {
		//Convert to HSV color space. 		
		Mat hsvImg = new Mat();
		hsvImg.create(input.size(), CvType.CV_8U);
		Imgproc.cvtColor(input, hsvImg, Imgproc.COLOR_BGR2HSV);

		//Find Blue cone in range.
		Mat blueConeTrd = new Mat();
		Core.inRange(hsvImg, blueConeL, blueConeH, blueConeTrd);

		//Red cone in range
		Mat redConeTrd1 = new Mat();
		Core.inRange(hsvImg, redConeL1, redConeH1, redConeTrd1);
		Mat redConeTrd2 = new Mat();
		Core.inRange(hsvImg, redConeL2, redConeH2, redConeTrd2);

		//Yellow pole in range
		Mat yellowPolTrd = new Mat ();
		Core.inRange(hsvImg, yellowPoleL, yellowPoleH, yellowPolTrd);

		//Remove background with low saturation
		Mat backgroundTrd = new Mat();
		Core.inRange(hsvImg, backgroundL, backgroundH, backgroundTrd);
		//Remove dark backgroud
		Mat darkTrd = new Mat();
		Core.inRange(hsvImg, darkL, darkH, darkTrd);

		//Combine these mask together
		Mat tobeRemoveRev = new Mat();
		Core.bitwise_or(blueConeTrd,redConeTrd1,tobeRemoveRev);
		Core.bitwise_or(redConeTrd2,tobeRemoveRev,tobeRemoveRev);
		Core.bitwise_or(yellowPolTrd,tobeRemoveRev,tobeRemoveRev);
		Core.bitwise_or(backgroundTrd,tobeRemoveRev,tobeRemoveRev);
		Core.bitwise_or(darkTrd,tobeRemoveRev,tobeRemoveRev);

		//Get reversed Mask
		Mat tobeRemove = new Mat();
		Core.bitwise_not(tobeRemoveRev,tobeRemove);

		///Apply mask to input image, and copy to new image. 
		Mat result  = new Mat(input.size(), CvType.CV_8UC3, new Scalar(255, 255, 255));
		input.copyTo(result, tobeRemove);

		Mat hsvResult = new Mat();
		Imgproc.cvtColor(result, hsvResult, Imgproc.COLOR_BGR2HSV);

		List<Mat> hsvResultPlane = new ArrayList<>();
		Core.split(hsvResult , hsvResultPlane);

		//Calculate the Hue average
		meanVal = calHueAvg(hsvResult);

		if (debug) {
			//Imgcodecs.imwrite( fileProcessing.getParent() + "\\result\\" + fileProcessing.getName() + "-thresh.jpg", tobeRemoveRev);
			Imgcodecs.imwrite("/sdcard/FIRST/Cone-result.jpg", result);
			//Imgcodecs.imwrite(fileProcessing.getParent() + "\\result\\" + fileProcessing.getName() + "-bgTrd.jpg", backgroundTrd);
		}

		return result;
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


	/**
	 * THis operation get SleveSide, according to the mean value calculated.
	 * @return
	 */
	public SleeveSide getSleveSide () {
		if ( meanVal > 0  ) {
			//Calculate the diff to peak
			double diff1 = Math.abs(meanVal - sleev1Peak);
			double diff2 = Math.abs(meanVal - sleev2Peak);
			double diff3 = Math.abs(meanVal - sleev3Peak);

			double smallest = Math.min(diff1, Math.min(diff2, diff3));
			if ( smallest == diff1) {
				return SleeveSide.Sleev1;
			} else if (smallest == diff2 ){
				return SleeveSide.Sleev2;

			} else if ( smallest == diff3 ) {
				return SleeveSide.Sleev3;
			} else {
				return SleeveSide.Unkown;
			}


		} else {
			return SleeveSide.Unkown;
		}

	}


}
