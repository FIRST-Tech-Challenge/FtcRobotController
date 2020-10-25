package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;

import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import static org.opencv.imgproc.Imgproc.*;

public class RingDetection extends OpenCvPipeline {
    Telemetry telemetry;
    Mat mat = new Mat();

    public static final int COLORCONST = 0;
    public static final double DISKR = 0.2;
    public static final double DMAXR = DISKR * 1.3;
    public static final double DMINR = DISKR * 0.7;
    public static final double STICKR = 4;
    public static final double SMAXR = STICKR * 1.45;
    public static final double SMINR = STICKR * 0.7;

    public RingDetection(Telemetry t){
        telemetry = t;
    }

    @Override
    public Mat processFrame(Mat input){
        mat = mark_rings(picSetup(input));
        return mat;
    }

    public static Mat picSetup(String file_name) {
        Mat raw = Imgcodecs.imread(file_name);
        Mat recolored = new Mat();
        cvtColor(raw, recolored, COLOR_RGB2BGR);
        Mat resized = new Mat();
        double scale = 960.0/raw.height();
        Imgproc.resize(recolored, resized, new Size(Math.round(raw.width() * scale), Math.round(raw.height() * scale)));
        return resized;
    }

    public static Mat picSetup(Mat input) {
        Mat recolored = new Mat();
        cvtColor(input, recolored, COLOR_RGB2BGR);
        Mat resized = new Mat();
        double scale = 960.0/input.height();
        Imgproc.resize(recolored, resized, new Size(Math.round(input.width() * scale), Math.round(input.height() * scale)));
        return resized;
    }


    public static Mat copyHSV(Mat input){
        Mat recolored = new Mat();
        cvtColor(input, recolored, COLOR_BGR2HSV);
        return recolored;
    }

    public static int avgValue(Mat input){
        int total = 0;
        int area = input.height() * input.width();
        for(int y = 0; y < input.height(); y++){
            for(int x = 0; x < input.width(); x++){
                total += input.get(y, x)[2];
            }
        }
        return (int)(total/area);
    }

    public static Mat find_yellows(Mat input){
        Mat threshold = new Mat();
        Core.inRange(input, new Scalar(7 + COLORCONST, 67, 90), new Scalar(20 + COLORCONST, 255, 255), threshold);
        return threshold;
    }

    public static Mat find_blues(Mat input){
        Mat threshold = new Mat();
        Core.inRange(input, new Scalar(95, 95, 50), new Scalar(135, 255, 190), threshold);
        return threshold;
    }

    public static Mat find_reds(Mat input){
        Mat threshold = new Mat();
        Core.inRange(input, new Scalar(8, 95, 100), new Scalar(35, 255, 255), threshold);
        return threshold;
    }

    public static ArrayList<Rect> find_subcontours(Mat input, Rect main){
        Mat threshold = find_yellows(copyHSV(input));
        Mat gray = new Mat();
        Mat grayblur = new Mat();
        Mat edgesX = new Mat();
        Mat edgesY = new Mat();
        Mat edges = new Mat();
        Mat edgesFinal = new Mat();
        Mat sub = new Mat();
        Mat preFilter = input.clone();

        ArrayList<Rect> output = new ArrayList<>();
        cvtColor(input, gray, COLOR_BGR2GRAY);
        Imgproc.blur(gray, grayblur, new Size(5,5));
        Imgproc.Sobel(grayblur, edgesX, -1, 0, 1);
        Imgproc.Sobel(grayblur, edgesY, -1, 1, 0);
        Core.add(edgesX, edgesY, edges);
        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 1));
        Core.inRange(edges, new Scalar(25, 25, 25), new Scalar(255, 255, 255), edgesFinal);
        Imgproc.dilate(edgesFinal, edgesFinal, kernel);
        Core.subtract(threshold, edgesFinal, sub);
        Core.inRange(sub, new Scalar(150, 150, 150), new Scalar(255, 255, 255), sub);
        List<MatOfPoint> contours = new ArrayList<>();

        Imgproc.findContours(sub, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            double r = (double) rect.height/rect.width;
            return ((r > DMAXR) || (r < DMINR) || (rect.width < main.width * 0.82));
        });

        for(MatOfPoint contour: contours){
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
            output.add(rect);
        }
        threshold.release();
        gray.release();
        grayblur.release();
        edgesX.release();
        edgesY.release();
        edges.release();
        edgesFinal.release();
        sub.release();
        preFilter.release();

        return output;
    }

    public static int closeIn(double[][] list, double x, double y, double width, double height, double epsilonW, double epsilonH){
        for(int i = 0; i< list.length; i++){
            if(Math.abs(y - list[i][4]) < epsilonH/2 && Math.abs(x - list[i][1]) < epsilonW/2){
                return -2;
            }
            if(Math.abs(list[i][1] - x) < epsilonW && Math.abs(list[i][2] - width) < epsilonW && Math.abs(list[i][5] - height) < epsilonH){
                return i;
            }
        }
        return -1;
    }

    public static Mat mark_rings(Mat input){
        List<MatOfPoint> contours = new ArrayList<>();

        Mat threshold = find_yellows(copyHSV(input));
        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);

        //initial filtering
        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            return (rect.area() < 4000) || (rect.height > rect.width);
        });

        //finding stacked rectangles and marking
        List<MatOfPoint> finalRects = new ArrayList<>();
        double[][] rectsData = new double[contours.size()][6];
        int used = 0;

        //sorting "rings" into stacks

        for(MatOfPoint contour:contours){
            Rect rect = Imgproc.boundingRect(contour);
            int newX = (int)Math.max(rect.x - (rect.width*0.3), 0);
            int newY = (int)Math.max(rect.y - (rect.height*0.1), 0);
            int newW = (int)Math.min((rect.width*1.3) + rect.x, input.width()) - newX;
            int newH = (int)Math.min((rect.height*1.1) + rect.y, input.height()) - newY;
            ArrayList<Rect> moreRects = find_subcontours(new Mat(input.clone(), new Rect(newX, newY, newW, newH)), rect);
            for(Rect subrect: moreRects){
                double epsilonW = rect.width * 0.3;
                double epsilonH = rect.height * 0.3;
                subrect.x += newX;
                subrect.y += newY;
                int index = closeIn(rectsData, subrect.x, subrect.y, subrect.width, subrect.height, epsilonW, epsilonH);
                if(index == -2){
                    System.out.println("Overlap");
                }
                else if(index == -1){

                    rectsData[used][0] = 1;
                    rectsData[used][1] = subrect.x;
                    rectsData[used][2] = subrect.width;
                    rectsData[used][3] = subrect.y + subrect.height;
                    rectsData[used][4] = subrect.y;
                    rectsData[used][5] = subrect.height;
                    used += 1;
                }
                else{
                    double occ = rectsData[index][0];
                    double avgX = rectsData[index][1];
                    double avgW = rectsData[index][2];
                    double maxY = rectsData[index][3];
                    double minY = rectsData[index][4];
                    double avgH = rectsData[index][5];
                    rectsData[index][0] += 1;
                    rectsData[index][1] = (subrect.x + occ*avgX)/(occ + 1);
                    rectsData[index][2] = (subrect.width + occ*avgW)/(occ + 1);
                    rectsData[index][3] = Math.max(subrect.y + subrect.height, maxY);
                    rectsData[index][4] = Math.min(subrect.y, minY);
                    rectsData[index][5] = (subrect.height + occ*avgH)/(occ + 1);
                }
            }
        }

        System.out.println(Arrays.deepToString(rectsData));

        //labeling found stacks
        for(double[] data: rectsData){
            if(data[0] > 0){
                Rect rect = new Rect((int) data[1], (int) data[4], (int) data[2], (int) (data[3] - data[4]));
                Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
                Imgproc.putText(input, "" + (int)data[0], new Point(data[1] + data[2]/2, data[4]), Imgproc.FONT_HERSHEY_SIMPLEX, 1, new Scalar(255, 0, 0), 2);
            }
        }
        return input;
    }

    public static Boolean wobble_stick(Mat input, MatOfPoint contour){
        Rect rect = Imgproc.boundingRect(contour);
        int newX = (int)Math.max(rect.x - (rect.width*0.1), 0);
        int newY = (int)Math.max(rect.y - (rect.height*0.1), 0);
        int newW = (int)Math.min((rect.width*1.1) + rect.x, input.width()) - newX;
        int newH = (int)Math.min((rect.height*0.9) + rect.y, input.height()) - newY;
        Mat cropped = new Mat(input.clone(), new Rect(newX, newY, newW, newH));
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(input, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);
        //initial filtering
        contours.removeIf(m -> {
            Rect subrect = Imgproc.boundingRect(m);
            double r = (double) subrect.height/subrect.width;
            return ((subrect.area() < 1000) || (r > SMAXR) || (r < SMINR));
        });
        return contours.size() > 0;
    }

    public static Mat mark_wobble(Mat input, String side){
        List<MatOfPoint> contours = new ArrayList<>();
        Mat threshold = new Mat();
        if(side.equals("blue")){
            threshold = find_blues(copyHSV(input));
        }
        else if(side.equals("red")){
            threshold = find_reds(copyHSV(input));
        }
        else{
            System.out.println("Not a valid side.");
        }

        Imgproc.findContours(threshold, contours, new Mat(), Imgproc.CHAIN_APPROX_NONE, Imgproc.CHAIN_APPROX_SIMPLE);
        final Mat thresh = threshold.clone();

        //initial filtering
        contours.removeIf(m -> {
            Rect rect = Imgproc.boundingRect(m);
            double r = (double) rect.height/rect.width;
            return ((rect.area() < 10000) || (rect.width > rect.height)) || !wobble_stick(thresh, m);
        });

        //labeling found wobble
        for(MatOfPoint wobble: contours){
            Rect rect = Imgproc.boundingRect(wobble);
            Imgproc.rectangle(input, rect.tl(), rect.br(), new Scalar(0, 255, 255), 2);
        }

        return input;
    }

}
