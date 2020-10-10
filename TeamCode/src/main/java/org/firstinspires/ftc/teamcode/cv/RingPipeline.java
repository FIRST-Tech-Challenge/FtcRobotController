//package org.firstinspires.ftc.teamcode.cv;
//
//import org.opencv.core.*;
//import org.opencv.core.Point;
//import org.opencv.highgui.HighGui;
//import org.opencv.imgcodecs.Imgcodecs;
//import org.opencv.imgproc.Imgproc;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class RingPipeline {
//    static {
//        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
//    }
//
//    static Mat hsv = new Mat();
//    static Mat inRange = new Mat();
//    static Mat region = new Mat();
//    static Mat edge = new Mat();
//    static Mat val = new Mat();
//    static Mat bin = new Mat();
//    static Mat divided = new Mat();
//    static Mat showy = new Mat();
//
//    static Mat elem = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 1), new Point(3, 0));
//    static Mat none = new Mat();
//
//    //testing dirty fix
//    static String currName = "";
//
//    public static void main(String[] args) {
//        //cv.Controls c = new cv.Controls();
//
//        var files = new String[]{"1.jpg", "2.jpg", "3.jpg", "4.jpg", "5.jpg", "6.jpg"
//        , "e1.png", "e2.png", "e3.png", "e4.png"};
//
//        // 1 - 6
//        for(String name : files) {
//            String url = "D:/Onedrive/Desktop/cv/" + name;
//            currName = name;
//            System.out.println(url);
//            Mat in = Imgcodecs.imread(url);
//
//            process(in);
//
//            //HighGui.waitKey();
//            in.release();
//        }
//
//        System.exit(0);
//
//    }
//    public static List<RotatedRect> process(Mat in){
//
//        Imgproc.resize(in, in, new Size(640, 480), 0, 0, Imgproc.INTER_LINEAR);
//
//            HighGui.imshow("original", in);
//
//        //happens once per image size
//        if(showy.cols() != in.cols() || showy.rows() != in.rows())
//            showy = new Mat(in.rows(), in.cols(),CvType.CV_8UC3);
//
//        //to hsv
//        Imgproc.cvtColor(in, hsv, Imgproc.COLOR_BGR2HSV);
//
//            //showHistogram(hsv, false);
//
//        //yellow range
//        Core.inRange(hsv, new Scalar(6, 62, 89), new Scalar(15, 255, 255), inRange);
//        Imgproc.cvtColor(inRange, inRange, Imgproc.COLOR_GRAY2BGR);
//
//            //HighGui.imshow("inRange", inRange);
//
//        //Keep in range regions only
//        Core.bitwise_and(inRange, hsv, region);
//
//        //edges
//        sobel(region, edge);
//
//        //extract value component
//        val = getChannel(edge, 2);
//
//            //HighGui.imshow("val", val.clone());
//
//        //binarize the value of edges
//        bin = new Mat();
//        Core.inRange(val, new Scalar(150), new Scalar(255), bin);
//
//            HighGui.imshow("binary val", bin);
//
//        //dilate binarized edges
//        //TODO Use Hough Transform (the world is not ready yet)
//        Imgproc.dilate(bin, bin, elem);
//
//        //hough(bin);
//
//        //split region of interest by edges
//        Imgproc.cvtColor(region, region, Imgproc.COLOR_BGR2GRAY);
//        Core.subtract(region, bin, divided);
//
//            //HighGui.imshow("divided", divided);
//
//        //find contours
//        List<MatOfPoint> contours = new ArrayList<>();
//        Imgproc.findContours(divided, contours, none, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
//
//        //filter contours
//        filterContours(contours, 70);
//
//        //draw contours
//        Imgproc.rectangle(showy, new Point(0,0), new Point(showy.cols(), showy.rows()), new Scalar(0,0,0), -1);
//        drawContours(contours, showy);
//
//            HighGui.imshow("showy", showy);
//            Imgcodecs.imwrite("Output " + currName, showy);
//
//        // bounding boxes
//        List<RotatedRect> bb = new ArrayList<>();
//        for(MatOfPoint c:contours){
//            //to matofpoint2f
//            bb.add(Imgproc.minAreaRect(new MatOfPoint2f(c.toArray())));
//        }
//
//        return bb;
//    }
//
//    public static void filterContours(List<MatOfPoint> inp, double size){
//        for(int i=0;i<inp.size();i++){
//            if(Imgproc.contourArea(inp.get(i)) < size){
//                inp.remove(i);
//                i--;
//            }
//        }
//    }
//
//    public static void drawContours(List<MatOfPoint> inp, Mat showy){
//        System.out.printf("there are %d contours%n",inp.size());
//
//        for(int i = 0;i<inp.size();i++) {
//            double rand = Math.random() * 255;
//            double rand2 = Math.random() * 255;
//            double rand3 = Math.random() * 255;
//            Imgproc.drawContours(showy, inp, i, new Scalar(rand, rand2, rand3>128?128:rand3), -1);
//        }
//    }
//
//    //Not memory safe
//    public static void distance (Mat inp, double thresh, Mat out){
//        //distance transform
//
//        out = new Mat(inp.rows(), inp.cols(), CvType.CV_8UC1);
//        Imgproc.distanceTransform(inp, out, Imgproc.DIST_C, 3);
//        // [70, 255]
//        Imgproc.threshold(out, out, thresh, 255, Imgproc.THRESH_BINARY);
//        out.convertTo(out, CvType.CV_8UC1);
//    }
//
//    public static void sum(Mat inp, Mat out){
//        List<Mat> channels = new ArrayList<>();
//        Core.split(inp, channels);
//        Mat sum = new Mat();
//
//        Core.add(channels.get(0), channels.get(1), sum);
//        Core.add(channels.get(2), sum, sum);
//
//        Core.multiply(sum, new Scalar(1.0/3), out);
//
//        sum.release();
//    }
//    public static Mat getChannel(Mat inp, int channel){
//        List<Mat> channels = new ArrayList<>();
//        Core.split(inp, channels);
//        return channels.get(channel);
//    }
//
//    public static void sobel(Mat inp, Mat edge){
//        Mat edgeY = new Mat();
//        Imgproc.Sobel(inp, edgeY, -1, 0, 1, 3, 1);
//
//        Mat edgeX = new Mat();
//        Imgproc.Sobel(inp, edgeX, -1, 0, 1, 3, 1);
//
//        Core.addWeighted(edgeX, 0.5, edgeY, 0.5, 0, edge);
//
//        edgeY.release();
//        edgeX.release();
//    }
//
//    public static void hough(Mat gray){
//        // Changing the color of the canny
//        Mat canvas = new Mat();
//        Imgproc.cvtColor(gray, canvas, Imgproc.COLOR_GRAY2BGR);
//
//        //Detecting the hough lines from (canny)
//        Mat lines = new Mat();
//        Imgproc.HoughLines(gray, lines, 1, Math.PI/180, 70);
//
//        //draw lines
//        for (int i = 0; i < lines.rows(); i++) {
//            double[] data = lines.get(i, 0);
//            double rho = data[0];
//            double theta = data[1];
//            double a = Math.cos(theta);
//            double b = Math.sin(theta);
//            double x0 = a*rho;
//            double y0 = b*rho;
//            //Drawing lines on the image
//            Point pt1 = new Point();
//            Point pt2 = new Point();
//            pt1.x = Math.round(x0 + 1000*(-b));
//            pt1.y = Math.round(y0 + 1000*(a));
//            pt2.x = Math.round(x0 - 1000*(-b));
//            pt2.y = Math.round(y0 - 1000 *(a));
//            Imgproc.line(canvas, pt1, pt2, new Scalar(0, 0, 255), 1);
//        }
//
//        HighGui.imshow("res", canvas);
//    }
//
//
//    /**
//     * Compute and show the histogram for the given {@link Mat} image
//     *
//     * @param frame
//     *            the {@link Mat} image for which compute the histogram
//     * @param gray
//     *            is a grayscale image?
//     */
//    static void showHistogram(Mat frame, boolean gray)
//    {
//        // split the frames in multiple images
//        List<Mat> images = new ArrayList<Mat>();
//        Core.split(frame, images);
//
//        // set the number of bins at 256
//        MatOfInt histSize = new MatOfInt(256);
//        // only one channel
//        MatOfInt channels = new MatOfInt(0);
//        // set the ranges
//        MatOfFloat histRange = new MatOfFloat(0, 256);
//
//        // compute the histograms for the B, G and R components
//        Mat hist_b = new Mat();
//        Mat hist_g = new Mat();
//        Mat hist_r = new Mat();
//
//        // B component or gray image
//        Imgproc.calcHist(images.subList(0, 1), channels, new Mat(), hist_b, histSize, histRange, false);
//
//        // G and R components (if the image is not in gray scale)
//        if (!gray)
//        {
//            Imgproc.calcHist(images.subList(1, 2), channels, new Mat(), hist_g, histSize, histRange, false);
//            Imgproc.calcHist(images.subList(2, 3), channels, new Mat(), hist_r, histSize, histRange, false);
//        }
//
//        // draw the histogram
//        int hist_w = 150; // width of the histogram image
//        int hist_h = 150; // height of the histogram image
//        int bin_w = (int) Math.round(hist_w / histSize.get(0, 0)[0]);
//
//        Mat histImage = new Mat(hist_h, hist_w, CvType.CV_8UC3, new Scalar(0, 0, 0));
//        // normalize the result to [0, histImage.rows()]
//        Core.normalize(hist_b, hist_b, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
//
//        // for G and R components
//        if (!gray)
//        {
//            Core.normalize(hist_g, hist_g, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
//            Core.normalize(hist_r, hist_r, 0, histImage.rows(), Core.NORM_MINMAX, -1, new Mat());
//        }
//
//        // effectively draw the histogram(s)
//        for (int i = 1; i < histSize.get(0, 0)[0]; i++)
//        {
//            // B component or gray image
//            Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_b.get(i - 1, 0)[0])),
//                    new Point(bin_w * (i), hist_h - Math.round(hist_b.get(i, 0)[0])), new Scalar(255, 0, 0), 2, 8, 0);
//            // G and R components (if the image is not in gray scale)
//            if (!gray)
//            {
//                Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_g.get(i - 1, 0)[0])),
//                        new Point(bin_w * (i), hist_h - Math.round(hist_g.get(i, 0)[0])), new Scalar(0, 255, 0), 2, 8,
//                        0);
//                Imgproc.line(histImage, new Point(bin_w * (i - 1), hist_h - Math.round(hist_r.get(i - 1, 0)[0])),
//                        new Point(bin_w * (i), hist_h - Math.round(hist_r.get(i, 0)[0])), new Scalar(0, 0, 255), 2, 8,
//                        0);
//            }
//        }
//
//        // display the histogram...
//        HighGui.imshow("histogram", histImage);
//
//    }
//
//}
