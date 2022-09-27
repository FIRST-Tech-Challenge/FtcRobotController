package org.firstinspires.ftc.teamcode.Framework.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

class AprilTagDetector extends OpenCvPipeline {
	private long nativeApriltagPtr;
	private Mat grey = new Mat();
	private volatile ArrayList<AprilTagDetection> detections = new ArrayList<>();

	double fx;
	double fy;
	double cx;
	double cy;

	boolean drawBoxes;

	// UNITS ARE METERS
	double tagsize;
	double tagsizeX;
	double tagsizeY;

	public AprilTagDetector(double tagsize, double fx, double fy, double cx, double cy, boolean draw) {
		this.tagsize = tagsize;
		this.tagsizeX = tagsize;
		this.tagsizeY = tagsize;
		this.fx = fx;
		this.fy = fy;
		this.cx = cx;
		this.cy = cy;
		this.drawBoxes = draw;

		// Allocate a native context object. See the corresponding deletion in the finalizer
		nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector("tag16h7", 3, 3);
	}

	public AprilTagDetector(double tagsize, boolean draw) {
		this(tagsize, 578, 578, 402, 222, draw);
	}

	public AprilTagDetector(double tagsize) {
		this(tagsize, false);
	}

	@Override
	public void finalize() {
		// Might be null if createApriltagDetector() threw an exception
		if(nativeApriltagPtr != 0) {
			// Delete the native context we created in the constructor
			AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
			nativeApriltagPtr = 0;
		}
		else {
			System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
		}
	}

	@Override
	public Mat processFrame(Mat input) {
		// Convert to greyscale
		Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

		// Run AprilTag
		detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);

		if (drawBoxes) {
			for (int i = 0; i < detections.size(); i++) {
				addBox(input, detections.get(i).corners);
			}
		}

		return input;
	}

	public ArrayList<AprilTagDetection> getLatestDetections() {
		return detections;
	}

	private void addBox(Mat image, Point[] points) {
		for (int i = 0; i < 4; i++) {
			Imgproc.line(image, points[i], points[(i + 1) % 4], new Scalar(255, 255, 0));
		}
	}
}