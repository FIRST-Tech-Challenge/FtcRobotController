package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.Locale;

/* PowerPlay image procesing pipeline to be run upon receipt of each frame from the camera.
 * Note that the processFrame() method is called serially from the frame worker thread -
 * that is, a new camera frame will not come in while you're still processing a previous one.
 * In other words, the processFrame() method will never be called multiple times simultaneously.
 *
 * However, the rendering of your processed image to the viewport is done in parallel to the
 * frame worker thread. That is, the amount of time it takes to render the image to the
 * viewport does NOT impact the amount of frames per second that your pipeline can process.
 *
 * IMPORTANT NOTE: this pipeline is NOT invoked on your OpMode thread. It is invoked on the
 * frame worker thread. This should not be a problem in the vast majority of cases. However,
 * if you're doing something weird where you do need it synchronized with your OpMode thread,
 * then you will need to account for that accordingly.
 */
class PipelinePowerPlay extends OpenCvPipeline
{
    /*
     * NOTE: if you wish to use additional Mat objects in your processing pipeline, it is
     * highly recommended to declare them here as instance variables and re-use them for
     * each invocation of processFrame(), rather than declaring them as new local variables
     * each time through processFrame(). This removes the danger of causing a memory leak
     * by forgetting to call mat.release(), and it also reduces memory pressure by not
     * constantly allocating and freeing large chunks of memory.
     */
    ArrayList<Mat> channels = new ArrayList<>(3);
    private Mat r    = new Mat();
    private Mat g    = new Mat();
    private Mat b    = new Mat();
    private int max;
    public static int avgR;
    public static int avgG;
    public static int avgB;
    private Point marker = new Point();        // Team Element (populated once we find it!)
    private Point sub1PointA;
    private Point sub1PointB;

    // Public statics to be used by opMode
    public static int signalZone;

    public static Mat finalAutoImage = new Mat();

    private final boolean redAlliance;
    private final boolean terminalSide;
    private static String directory;

    PipelinePowerPlay(boolean redAlliance, boolean terminalSide) {
        // Create a subdirectory based on DATE
        String dateString = new SimpleDateFormat("yyyy-MM-dd", Locale.getDefault()).format(new Date());
        directory = Environment.getExternalStorageDirectory().getPath() + "//FIRST//Webcam//" + dateString;
        this.redAlliance = redAlliance;
        this.terminalSide = terminalSide;
        if(this.redAlliance) {
            if(this.terminalSide) {
                directory += "/red_terminal";
            } else {
                directory += "/red_not_terminal";
            }
        } else {
            if(this.terminalSide) {
                directory += "/blue_terminal";
            } else {
                directory += "/blue_not_terminal";
            }
        }
        sub1PointA = new Point( 157,90);  // 20x20 pixels on signal sleeve
        sub1PointB = new Point( 177,110);

        // Create the directory structure to store the autonomous image used to start auto.
        File autonomousDir = new File(directory);
        autonomousDir.mkdirs();
    }

    public static void saveLastAutoImage() {
        String timeString = new SimpleDateFormat("hh-mm-ss", Locale.getDefault()).format(new Date());
        String directoryPath = directory + "/" + "AutoImage_" + timeString + ".png";

        new Thread(new Runnable()
        {
            @Override
            public void run()
            {
                try
                {
                    Imgproc.cvtColor(finalAutoImage, finalAutoImage, Imgproc.COLOR_RGB2BGR);
                    Imgcodecs.imwrite(directoryPath, finalAutoImage);
                }
                catch (Exception e)
                {
                    e.printStackTrace();
                }
                finally
                {
                    finalAutoImage.release();
                }
            }
        }).start();
    }

    @Override
    public Mat processFrame(Mat input)
    {
        // Extract the RGB channels from the image frame
        Core.split(input, channels);

        // Pull RGB data for the sample zone from the RBG channels
        r = channels.get(0).submat(new Rect(sub1PointA,sub1PointB) );
        g = channels.get(1).submat(new Rect(sub1PointA,sub1PointB) );
        b = channels.get(2).submat(new Rect(sub1PointA,sub1PointB) );

        // Average the three sample zones
        avgR = (int)Core.mean(r).val[0];
        avgB = (int)Core.mean(b).val[0];
        avgG = (int)Core.mean(g).val[0];

        // Draw rectangles around the sample zone
        Imgproc.rectangle(input, sub1PointA, sub1PointB, new Scalar(0, 0, 255), 1);

        // Determine which RBG channel had the highest value
        max = Math.max(avgR, Math.max(avgB, avgG));
        // Draw a circle on the detected team shipping element
        marker.x = (sub1PointA.x + sub1PointB.x) / 2;
        marker.y = (sub1PointA.y + sub1PointB.y) / 2;
        if(max == avgR) {
            Imgproc.circle(input, marker, 5, new Scalar(255, 0, 0), -1);
            signalZone = 1;
        } else if(max == avgG) {
            Imgproc.circle(input, marker, 5, new Scalar(0, 255, 0), -1);
            signalZone = 2;
        } else if(max == avgB) {
            Imgproc.circle(input, marker, 5, new Scalar(0, 0, 255), -1);
            signalZone = 3;
        } else {
            signalZone = 3;
        }

        input.copyTo(finalAutoImage);

        // Free the allocated submat memory
        r.release();
        r = null;
        g.release();
        g = null;
        b.release();
        b = null;
        channels.get(0).release();
        channels.get(1).release();
        channels.get(2).release();

        return input;
    }
} // PipelinePowerPlay

