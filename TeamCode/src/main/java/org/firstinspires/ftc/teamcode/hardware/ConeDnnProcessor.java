package org.firstinspires.ftc.teamcode.hardware;

import java.io.File;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.dnn.Dnn;
import org.opencv.dnn.Net;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

/**
 * This class use OpenCV DNN module to detect Sleeve
 *
 */
public class ConeDnnProcessor extends OpenCvPipeline {

	/**
	 * Load net during constructx
	 */
	public ConeDnnProcessor() {
		net = Dnn.readNetFromTensorflow(
				"/sdcard/First/DNN/ssd_mobilenet_v3_large_coco_2020_01_14_frozen.pb",
				"/sdcard/First/DNN/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt");
		//"C:\\Transfer\\ssd_mobilenet_v1_coco_2017_11_17_frozen.pb",
		//"C:\\Transfer\\ssd_mobilenet_v1_coco_2017_11_17.pbtxt" );

	}

	Net net;

	private StringBuffer detectMsgBuf;
	private String detectMsg ;

	boolean debug = true;

	public static void main(String[] args) {
		System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
		ConeDnnProcessor proc = new ConeDnnProcessor();

		File imageDir = new File("C:\\FTC Code\\TestPIcs");
		File resultDir = new File("C:\\FTC Code\\TestPIcs\\result");

		File[] files = imageDir.listFiles();
		for (int i = 0; i < files.length; i++) {
			if (files[i].isFile()) {
				Mat test01 = Imgcodecs.imread(files[i].getAbsolutePath());
				Mat result = proc.processFrame(test01);
				System.out.println("Detected: " + proc.getDetectMsg());
				//Imgcodecs.imwrite(resultDir.getAbsolutePath() + "\\" + files[i].getName() + "-result.jpg", result);

			}
		}

	}


	public Mat processFrame(Mat input) {
		long startMills  = System.currentTimeMillis();
		detectMsgBuf = new StringBuffer();
		/**
		 * These values need to correspond to the Network used.
		 * Fast RCNN:  800 x 600,  Scale = 1.0 , mean 0
		 * SSD Mobilenet v1  300 X 300 , Scale 1.0 , mean 0
		 * SSD Mobilenet V3  320x320  Scale 1.0
		 */
		final int IN_WIDTH = 320;
		final int IN_HEIGHT = 320;

		final double IN_SCALE_FACTOR = 1.0;
		final double MEAN_VAL = 0;
		final double THRESHOLD = 0.5;

		//Create Blob, using the size of the Net, and also switch R and B chanel. as OpenCV read
		// Image as BRG, and Net is RBG
		Mat blob = Dnn.blobFromImage(input, IN_SCALE_FACTOR, new Size(IN_WIDTH, IN_HEIGHT),
				new Scalar(MEAN_VAL, MEAN_VAL, MEAN_VAL), true, /* crop */ false);

		net.setInput(blob);
		Mat detections = net.forward();

		detections = detections.reshape(1, (int) detections.total() / 7);
		
		int cols = input.cols();
		int rows = input.rows();

		for (int i = 0; i <= detections.rows(); ++i) {
			double[] conf = detections.get(i, 2);
			if (conf != null) {
				double confidence = conf[0];
				if (confidence > THRESHOLD) {
					int classId = (int) detections.get(i, 1)[0];
					String label = classNames[classId] + ": " + confidence;
					detectMsgBuf.append( label + " " );

					/*
					int left = (int) (detections.get(i, 3)[0] * cols);
					int top = (int) (detections.get(i, 4)[0] * rows);
					int right = (int) (detections.get(i, 5)[0] * cols);
					int bottom = (int) (detections.get(i, 6)[0] * rows);

					// Draw rectangle around detected object.
					Imgproc.rectangle(input, new Point(left, top), new Point(right, bottom), new Scalar(0, 255, 0));
					int[] baseLine = new int[1];
					Size labelSize = Imgproc.getTextSize(label, Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, 1, baseLine);
					// Draw background for label.
					Imgproc.rectangle(input, new Point(left, top - labelSize.height),
							new Point(left + labelSize.width, top + baseLine[0]), new Scalar(255, 255, 255),
							Imgproc.FILLED);
					// Write class name and confidence.
					Imgproc.putText(input, label, new Point(left, top), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
							new Scalar(0, 0, 0));
					*/
				}
			}
		}

		long endMills  = System.currentTimeMillis();
		detectMsgBuf.append(" Duration: " + (endMills - startMills) );
		detectMsg = detectMsgBuf.toString();

		return input;
	}

	public String getDetectMsg() {
		if (detectMsg!=null) {
			return detectMsg;
		} else {
			return "N1ONE";
		}
	}

	private static final String[] classNames = {"bck","person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
			"boat", "traffic light", "fire hydrant", "street sign", "stop sign", "parking meter", "bench", "bird", "cat", "dog", 
			"horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "hat", "backpack", "umbrella", "shoe", "eye glasses",
			"handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", 
			"skateboard", "surfboard", "tennis racket", "bottle", "plate", "wine glass", "cup", "fork", "knife", "spoon", "bowl", 
			"banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", 
			"potted plant", "bed", "mirror", "dining table", "window", "desk", "toilet", "door", "tv", "laptop", "mouse", "remote",
			"keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "blender", "book", "clock", "vase", 
			"scissors", "teddy bear", "hair drier", "toothbrush", "hair brush" };

}
