package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.*;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;

public class PipePoleTracker extends OpenCvPipeline {

    //TODO substitute the focusRect for the below testRect (focusRect should be the problem)
    Rect testRect = new Rect(new Point(0,0), new Point(300,300));


    Mat inputHSV = new Mat();
    Mat inputMask = new Mat();
    Mat inputOriginal = new Mat();
    Mat inputMaskOriginal = new Mat();
    Mat mat = new Mat();
    Mat focusSubMat = new Mat();



    Scalar white = new Scalar(0, 0, 0); // In grey scale
    Scalar grey = new Scalar(75, 0, 0); // In grey scale
    Scalar red = new Scalar(0, 0, 255); // in BGR
    Scalar blue = new Scalar(255, 0, 0); // in BGR
    Scalar green = new Scalar(0, 255, 0); // in BGR
    Scalar yellow = new Scalar(0,255,255); // in BGR

    double threshold = 0.4;





    PipePoleTracker(String level){
        levelString = level;
    }

    @Override
    public Mat processFrame(Mat input) {



        Imgproc.cvtColor(input,inputHSV,Imgproc.COLOR_BGR2HSV);
        Core.inRange(inputHSV, new Scalar(81, 115, 164), new Scalar(107, 255, 255), inputMask);






//        // Creating grid's rectangles
//        for(int i =0; i < gridY; i++){
//            for(int j = 0; j < gridX; j++){
//                Point rectTL = new Point(j*box_width,i*box_height);
//                Point rectBR = new Point((j+1)*box_width,(i+1)*box_height);
//                rectanglesGrid[i][j] = new Rect(rectTL,rectBR);
//                rectTL = new Point(j*box_width,i*box_height);
//                rectBR = new Point((j+1)*box_width,(i+1)*box_height);
//                rectanglesGridDraw[i][j] = new Rect(rectTL, rectBR);
//            }
//        }
//
//        // Creating grid's subMats
//        for(int i = 0; i < gridY; i++){
//            for(int j = 0; j < gridX; j++){
//                matsGrid[i][j] = inputMask.submat(rectanglesGrid[i][j]);
//            }
//        }



        int d = 0;
        int z = 0;



            //TODO The below color filtered is being done twice here
            Imgproc.cvtColor(input,inputHSV,Imgproc.COLOR_BGR2HSV);
            Core.inRange(inputHSV,  new Scalar(81, 115, 164), new Scalar(107, 255, 255), inputMask);


            //"input" is image operated on, whereas "inputOriginal" will be drawn on
            inputOriginal = input;
            inputMaskOriginal = inputMask;

//            percentColor = Core.countNonZero(inputMask); //TODO <-- This is wrong, this counts whole sum, not percent



            // Level1 and Level2 run the same grid code, so they are in the same 'grouping.' The only difference is the
            // beginning below where the region of interest (full image or focusRect) is established.
            if(levelString.equals("one") || levelString.equals("two")) {

                /**
                 * Level1 image assignment
                 */
                if(levelString.equals("one") && level1Assigment == true){

                    x_resolution = input.cols();
                    y_resolution = input.rows();

                    box_width = (int)(x_resolution/gridX);
                    box_height = (int)(y_resolution/gridY);

                    minimumWidth = box_width*5; //Pole SHOULD be 2 boxes wide, but focus rect adds 1.5 on each side
                    minimumHeight = box_height*11; //Pole SHOULD be 10 boxes high, but focus rect adds 0.5 on each side
                }

                if (levelString.equals("one") && level1Assigment == false) {

                    gridX = 20;
                    gridY = 20;

                    rectanglesGrid = new Rect[gridY][gridX];
                    rectanglesGridDraw = new Rect[gridY][gridX];
                    matsGrid = new Mat[gridY][gridX];
                    identifiedBoxesBoolean = new boolean[gridY][gridX];
                    centersX = new int[gridY][gridX];
                    centersXDraw = new int[gridY][gridX];
                    centersY = new int[gridY][gridX];
                    centersYDraw = new int[gridY][gridX];

                    x_resolution = input.cols();
                    y_resolution = input.rows();

                    box_width = (int)(x_resolution/gridX);
                    box_height = (int)(y_resolution/gridY);

                    minimumWidth = box_width*5; //Pole SHOULD be 2 boxes wide, but focus rect adds 1.5 on each side
                    minimumHeight = box_height*11; //Pole SHOULD be 10 boxes high, but focus rect adds 0.5 on each side

                    lowestX = (int)x_resolution;
                    highestX = 0;
                    lowestY = (int)y_resolution;
                    highestY = 0;

                    box_width = (int) (x_resolution / gridX);
                    box_height = (int) (y_resolution / gridY);



                    // Creating grid's rectangles
                    for (int i = 0; i < gridY; i++) {
                        for (int j = 0; j < gridX; j++) {
                            Point rectTL = new Point(j * box_width, i * box_height);
                            Point rectBR = new Point((j + 1) * box_width, (i + 1) * box_height);
                            rectanglesGrid[i][j] = new Rect(rectTL, rectBR);
                            rectTL = new Point(j * box_width, i * box_height);
                            rectBR = new Point((j + 1) * box_width, (i + 1) * box_height);
                            rectanglesGridDraw[i][j] = new Rect(rectTL, rectBR);
                        }
                    }

                    level1Assigment = true;
                    level2Assignment = false;
                    level3Assignment = false;
                }

                /**
                 * Level2 Assigment below
                 */
                //So basically when we move into level2, we want to keep reading the input region into the submat region
                // However each time the capture.read is run, it resizes "input" into the original resolution, SOOOOO
                //this little thing is so the submat is retained without rerunning all the below stuff
                if (levelString.equals("two") && level2Assignment == true) {

//                    gridX = 40;
//                    gridY = 40;


                    lowestX = 112;
                    highestX = 336;
                    focusRect = new Rect(new Point(lowestX, lowestY), new Point(highestX, highestY));

                    input = input.submat(focusRect);
                    inputMask = inputMask.submat(focusRect);

                    x_resolution = focusRect.width;
                    y_resolution = focusRect.height;

                    percentColor = ((Core.countNonZero(inputMask))/focusRect.area())*100;


                }

                if (levelString.equals("two") && level2Assignment == false && focusRect != null) {

                    gridX = 40;
                    gridY = 10;

                    rectanglesGrid = new Rect[gridY][gridX];
                    rectanglesGridDraw = new Rect[gridY][gridX];
                    matsGrid = new Mat[gridY][gridX];
                    identifiedBoxesBoolean = new boolean[gridY][gridX];
                    centersX = new int[gridY][gridX];
                    centersXDraw = new int[gridY][gridX];
                    centersY = new int[gridY][gridX];
                    centersYDraw = new int[gridY][gridX];

                    lowestX = 112;
                    highestX = 336;
                    focusRect = new Rect(new Point(lowestX, lowestY), new Point(highestX, highestY));

                    input = input.submat(focusRect);
                    inputMask = inputMask.submat(focusRect);

                    x_resolution = focusRect.width;
                    y_resolution = focusRect.height;

                    box_width = (int) (x_resolution / gridX);
                    box_height = (int) (y_resolution / gridY);

                    percentColor = ((Core.countNonZero(inputMask))/focusRect.area())*100;


                    // Creating grid's rectangles
                    for (int i = 0; i < gridY; i++) {
                        for (int j = 0; j < gridX; j++) {
                            Point rectTL = new Point(j * box_width, i * box_height);
                            Point rectBR = new Point((j + 1) * box_width, (i + 1) * box_height);
                            rectanglesGrid[i][j] = new Rect(rectTL, rectBR);
                            rectTL = new Point(lowestX + (j * box_width), lowestY + (i * box_height));
                            rectBR = new Point(lowestX + ((j + 1) * box_width), lowestY + ((i + 1) * box_height));
                            rectanglesGridDraw[i][j] = new Rect(rectTL, rectBR);
                        }
                    }


                    level2Assignment = true;
                    level1Assigment = false;
                    level3Assignment = false;
                }

//                boxBL_x = rectanglesGridDraw[gridY-1][gridX-1].x;
//                boxBL_y = rectanglesGridDraw[gridY-1][gridX-1].y;


                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        matsGrid[i][j] = inputMask.submat(rectanglesGrid[i][j]);
                    }
                }

                // Find all 'identified' boxes
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        double percentColor = (Core.countNonZero(matsGrid[i][j])) / rectanglesGrid[i][j].area();
//                        System.out.println("Percent Color of (" + i + ", " + j + ") is: " + percentColor);
//                        System.out.println("Number of Pixels at (" + i + ", " + j + ") is: " + Core.countNonZero(matsGrid[i][j]));

                        if (percentColor > threshold) {
                            identifiedBoxesBoolean[i][j] = true;
                        } else {
                            identifiedBoxesBoolean[i][j] = false;
                        }
                    }
                }
                //TODO could be cleaned up so that center arrays use the rectangles grid

                // Find all the X & Y Centers
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        centersX[i][j] = (int) (j * box_width + 0.5 * box_width);
                        centersY[i][j] = (int) (i * box_height + 0.5 * box_height);
                        centersXDraw[i][j] = (int) (rectanglesGridDraw[i][j].x + (0.5 * box_width));
                        centersYDraw[i][j] = (int) (rectanglesGridDraw[i][j].y + (0.5 * box_height));
                    }
                }

                // Find average X & Y points
                int totalX = 0;
                int totalY = 0;
                int amountIdentifiedBoxes = 0;
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        if (identifiedBoxesBoolean[i][j] == true) {
                            totalX += centersX[i][j];
                            totalY += centersY[i][j];
                            amountIdentifiedBoxes++;
                        } else {

                        }
                    }
                }

                Point[] identifiedBoxesPoints = new Point[amountIdentifiedBoxes];
                int[] identifiedBoxesX = new int[amountIdentifiedBoxes];
                int[] identifiedBoxesY = new int[amountIdentifiedBoxes];

                int temp = 0;
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        if (identifiedBoxesBoolean[i][j] == true) {
                            identifiedBoxesPoints[temp] = new Point(i, j);
                            identifiedBoxesX[temp] = j;
                            identifiedBoxesY[temp] = i;
                            temp++;
                        } else {

                        }
                    }
                }


                System.out.println(Arrays.toString(identifiedBoxesPoints));
                System.out.println(Arrays.toString(identifiedBoxesY));
                System.out.println(Arrays.toString(identifiedBoxesX));


                int randomCounter = 0;

                if (amountIdentifiedBoxes == 0) {
                    largestSize = 0;
                    amountIdentifiedBoxes = 1;
                    randomCounter = 1;
                }

                int avgX = (int) (totalX / amountIdentifiedBoxes);
                int avgY = (int) (totalY / amountIdentifiedBoxes);


                //Drawing the rectangles
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        Scalar rectGreyColor = white;
                        Scalar rectBGRColor = blue;
                        if (identifiedBoxesBoolean[i][j] == true) {
                            rectGreyColor = grey;
                            rectBGRColor = green;
                        }
                        Imgproc.rectangle(inputOriginal, rectanglesGridDraw[i][j], rectBGRColor, 2);
                        Imgproc.rectangle(inputMaskOriginal, rectanglesGridDraw[i][j], rectGreyColor, 2); //Only adjust the first channel in this scalar as it's a greyscale image
                    }


                }

                //  Draw out all the center points
                for (int i = 0; i < gridY; i++) {
                    for (int j = 0; j < gridX; j++) {
                        Imgproc.ellipse(inputMaskOriginal, new RotatedRect(new Point(centersXDraw[i][j], centersYDraw[i][j]), new Size(2, 2), 180), new Scalar(50, 0, 0), Imgproc.FILLED);
                    }
                }

                /**
                 * Differentiating Objects
                 */


                if (randomCounter == 0) {
                    ArrayList<ArrayList<Point>> objects = new ArrayList<>();
                    int objectCounter = 0;
                    int addedBoxes = 0;
                    int boxesCheckedLastLoop = 0;
                    int totalBoxesChecked = 0;

                    ArrayList<Point> allCheckedBoxes = new ArrayList<>();


                    // Creating all the separate objects (yay)
                    System.out.println(addedBoxes);
                    System.out.println(amountIdentifiedBoxes);


                    int currentI = identifiedBoxesY[0];
                    int currentJ = identifiedBoxesX[0];
                    ArrayList<Point> innerBoxes = new ArrayList<>(1);
                    ArrayList<Integer> innerBoxesX = new ArrayList<>();
                    ArrayList<Integer> innerBoxesY = new ArrayList<>();

                    innerBoxesY.add(identifiedBoxesY[0]);
                    innerBoxesX.add(identifiedBoxesX[0]);


                    innerBoxes.add(new Point(currentI, currentJ));
                    while (totalBoxesChecked < amountIdentifiedBoxes) {
                        System.out.println(" ------------ Starting another loop ");
                        System.out.println("Addedboxes: " + addedBoxes);
                        System.out.println("AmountIdentifiedBoxes: " + amountIdentifiedBoxes);

                        innerBoxes = new ArrayList<>();
                        innerBoxesX = new ArrayList<>();
                        innerBoxesY = new ArrayList<>();


                        for (int i = 0; i < amountIdentifiedBoxes; i++) {
                            if (!allCheckedBoxes.contains(identifiedBoxesPoints[i])) {
                                innerBoxesY.add(identifiedBoxesY[i]);
                                innerBoxesX.add(identifiedBoxesX[i]);
                                innerBoxes.add(new Point(identifiedBoxesY[i], identifiedBoxesX[i]));
                                break;
                            } else {

                            }
                        }


                        for (int i = 0; i < innerBoxes.size(); i++) {

                            currentI = innerBoxesY.get(i);
                            currentJ = innerBoxesX.get(i);

                            allCheckedBoxes.add(new Point(currentI, currentJ));

                            System.out.println("i: " + i);
                            System.out.println("Current innerBoxes.size() is: " + innerBoxes.size());
                            System.out.println("Current Coordinate: (" + currentI + ", " + currentJ + ")");


                            try {
                                if (identifiedBoxesBoolean[currentI - 1][currentJ - 1] == true && !innerBoxes.contains(new Point(currentI - 1, currentJ - 1))) {
                                    innerBoxes.add(new Point(currentI - 1, currentJ - 1));
                                    innerBoxesY.add(currentI - 1);
                                    innerBoxesX.add(currentJ - 1);

                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ - 1) + ") <-- True");
                                    addedBoxes++;

                                } else {
                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ - 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI - 1) + ", " + (currentJ - 1) + ") <-- Error");
                            }
                            try {
                                if (identifiedBoxesBoolean[currentI - 1][currentJ] == true && !innerBoxes.contains(new Point(currentI - 1, currentJ))) {
                                    innerBoxes.add(new Point(currentI - 1, currentJ));
                                    innerBoxesY.add(currentI - 1);
                                    innerBoxesX.add(currentJ);
                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI - 1) + ", " + (currentJ) + ") <-- Error");

                            }
                            try {
                                if (identifiedBoxesBoolean[currentI - 1][currentJ + 1] == true && !innerBoxes.contains(new Point(currentI - 1, currentJ + 1))) {
                                    innerBoxes.add(new Point(currentI - 1, currentJ + 1));
                                    innerBoxesY.add(currentI - 1);
                                    innerBoxesX.add(currentJ + 1);
                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ + 1) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI - 1) + ", " + (currentJ + 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI - 1) + ", " + (currentJ + 1) + ") <-- Error");

                            }

                            try {
                                if (identifiedBoxesBoolean[currentI][currentJ - 1] == true && !innerBoxes.contains(new Point(currentI, currentJ - 1))) {
                                    innerBoxes.add(new Point(currentI, currentJ - 1));
                                    innerBoxesY.add(currentI);
                                    innerBoxesX.add(currentJ - 1);
                                    System.out.println("(" + (currentI) + ", " + (currentJ - 1) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI) + ", " + (currentJ - 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI) + ", " + (currentJ - 1) + ") <-- Error");

                            }
                            try {
                                if (identifiedBoxesBoolean[currentI][currentJ + 1] == true && !innerBoxes.contains(new Point(currentI, currentJ + 1))) {
                                    innerBoxes.add(new Point(currentI, currentJ + 1));
                                    innerBoxesY.add(currentI);
                                    innerBoxesX.add(currentJ + 1);
                                    System.out.println("(" + (currentI) + ", " + (currentJ + 1) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI) + ", " + (currentJ + 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI) + ", " + (currentJ + 1) + ") <-- Error");

                            }


                            try {
                                if (identifiedBoxesBoolean[currentI + 1][currentJ - 1] == true && !innerBoxes.contains(new Point(currentI + 1, currentJ - 1))) {
                                    innerBoxes.add(new Point(currentI + 1, currentJ - 1));
                                    innerBoxesY.add(currentI + 1);
                                    innerBoxesX.add(currentJ - 1);
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ - 1) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ - 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI + 1) + ", " + (currentJ - 1) + ") <-- Error");

                            }
                            try {
                                if (identifiedBoxesBoolean[currentI + 1][currentJ] == true && !innerBoxes.contains(new Point(currentI + 1, currentJ))) {
                                    innerBoxes.add(new Point(currentI + 1, currentJ));
                                    innerBoxesY.add(currentI + 1);
                                    innerBoxesX.add(currentJ);
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI + 1) + ", " + (currentJ) + ") <-- Error");

                            }
                            try {
                                if (identifiedBoxesBoolean[currentI + 1][currentJ + 1] == true && !innerBoxes.contains(new Point(currentI + 1, currentJ + 1))) {
                                    innerBoxes.add(new Point(currentI + 1, currentJ + 1));
                                    innerBoxesY.add(currentI + 1);
                                    innerBoxesX.add(currentJ + 1);
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ + 1) + ") <-- True");
                                    addedBoxes++;
                                } else {
                                    System.out.println("(" + (currentI + 1) + ", " + (currentJ + 1) + ") <-- False or already in the array");
                                }
                            } catch (Exception e) {
                                System.out.println("(" + (currentI + 1) + ", " + (currentJ + 1) + ") <-- Error");

                            }
                            boxesCheckedLastLoop++;
                        }
                        objects.add(innerBoxes);
                        totalBoxesChecked += boxesCheckedLastLoop;
                        System.out.println("-------------- Inner loop cycle ending");
                        System.out.println("boxesCheckedLastLoop: " + boxesCheckedLastLoop);
                        System.out.println("addedBoxes: " + addedBoxes);
                        System.out.println("totalBoxesChecked: " + totalBoxesChecked);
                        boxesCheckedLastLoop = 0;
                        addedBoxes = 0;
                    }

                    System.out.println("objects.size(): " + objects.size());
                    System.out.println(objects);

                    if (levelString.equals("one")) {
                        lowestX = 0;
                        lowestY = 0;
                    }

                    Point[] objectCenters = new Point[objects.size()];
                    int[] objectSizes = new int[objects.size()];

                    for (int i = 0; i < objects.size(); i++) {
                        totalX = 0;
                        totalY = 0;
                        avgX = 0;
                        avgY = 0;
                        for (int j = 0; j < objects.get(i).size(); j++) {
                            totalX += centersX[(int) objects.get(i).get(j).x][(int) objects.get(i).get(j).y]; //Read below
                            totalY += centersY[(int) objects.get(i).get(j).x][(int) objects.get(i).get(j).y]; // The X & Y are inversed because are grid is done as (y,x), whereas the standard is (x,y)
                        }
                        avgX = totalX / objects.get(i).size();
                        avgY = totalY / objects.get(i).size();
                        objectCenters[i] = new Point(lowestX + avgX, lowestY + avgY); //objectCenters is in (x,y) as per OpenCV standard
                        objectSizes[i] = objects.get(i).size();
                        Imgproc.ellipse(inputOriginal, new RotatedRect(objectCenters[i], new Size(8, 8), 180), yellow, Imgproc.FILLED);
                    }


                    System.out.println("Objects' Centers: " + Arrays.toString(objectCenters));
                    System.out.println("Objects' Size: " + Arrays.toString(objectSizes));
                    System.out.println(objects);


                    //Find the biggest object and add a blue dot just to track it
                    int indexOfBiggest = 0;
                    largestSize = 0;
                    for (int i = 0; i < objects.size(); i++) {
                        if (objectSizes[i] > largestSize) {
                            largestSize = objectSizes[i];
                            indexOfBiggest = i;
                        }
                    }

                    centerX = (int)objectCenters[indexOfBiggest].x;

                    Imgproc.ellipse(inputOriginal, new RotatedRect(objectCenters[indexOfBiggest], new Size(8, 8), 180), blue, Imgproc.FILLED);

                    //Y and X are inverted for the grid.... my bad
                    largestObjectWidth = 0;
                    largestObjectLowestX = gridX;
                    largestObjectHighestX = 0;
                    for(int i = 0; i < objectSizes[indexOfBiggest]; i++) {
                        if (objects.get(indexOfBiggest).get(i).y < largestObjectLowestX) {
                            largestObjectLowestX = (int) objects.get(indexOfBiggest).get(i).y;
                        }
                        if (objects.get(indexOfBiggest).get(i).y > largestObjectHighestX) {
                            largestObjectHighestX = (int) objects.get(indexOfBiggest).get(i).y;
                        } else {

                        }
                    }
                    largestObjectWidth = largestObjectHighestX - largestObjectLowestX + 1;

                    //If we are in level one, we continue drawing a rectangle around the largest object (prep for level 2 & 3)
                    if (levelString.equals("one")) {

//                        focusRect = null; //TODO may be source of rectangle carry over despite no yellow in image (shadow rect)

                        lowestX = (int) x_resolution;
                        highestX = 0;
                        lowestY = (int) y_resolution;
                        highestY = 0;

                        for (int i = 0; i < objects.get(indexOfBiggest).size(); i++) {
                            if (centersX[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y] < lowestX) {
                                lowestX = centersX[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y];
                                System.out.println("New lowestX found!");
                            }
                            if (centersX[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y] > highestX) {
                                highestX = centersX[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y];
                                System.out.println("New highestX found!");

                            }

                            if (centersY[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y] < lowestY) {
                                lowestY = centersY[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y];
                                System.out.println("New lowestY found!");

                            }
                            if (centersY[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y] > highestY) {
                                highestY = centersY[(int) objects.get(indexOfBiggest).get(i).x][(int) objects.get(indexOfBiggest).get(i).y];
                                System.out.println("New highestY found!");
                            }
                        }

                        //Make new "focus" submat for increased resolution (level2, and technically for level3)
                        if ((lowestX - 2 * box_width) < 0) {
                            lowestX = 0;
                        } else {
                            lowestX = lowestX - 2 * box_width;
                        }

                        if ((highestX + 2 * box_width) > x_resolution) {
                            highestX = (int) x_resolution;
                        } else {
                            highestX = highestX + 2 * box_width;
                        }

                        if ((lowestY - box_height) < 0) {
                            lowestY = 0;
                        } else {
                            lowestY = lowestY - box_height;
                        }

                        if (highestY + box_height > y_resolution) {
                            highestY = y_resolution;
                        } else {
                            highestY = highestY + box_height;
                        }



                        focusRect = new Rect(new Point(lowestX, lowestY), new Point(highestX, highestY));


                    }


                }

                System.out.println("--------------------lowestX: " + lowestX);
                System.out.println("highestX: " + highestX);
                System.out.println("lowestY: " + lowestY);
                System.out.println("--------------------highestY " + highestY);
                System.out.println("focusRect.x: " + focusRect.x);


            }
            //There are 4 requirements for level3: level2 is true, robot is aligned, robot is close enough, and the imu has recorded a value


            //This is level 3 below. It expands the focusRect vertically, then reads the percent of color in the total box.
            else if (levelString.equals("three")){

//                percentColor = 0;


//                x_resolution = input.cols();


//                input = input.submat(focusRect);
//                inputMask = inputMask.submat(focusRect);

                focusRect2 = new Rect(new Point(50,0), new Point(400, inputOriginal.rows()));

                input = input.submat(focusRect2);
                inputMask = inputMask.submat(focusRect2);

                percentColor = ((Core.countNonZero(inputMask))/focusRect2.area())*100;

                Imgproc.rectangle(inputOriginal, focusRect2, red, 2);

//                if(percentColor < 0.1){
//                    System.out.println("STOP! You've reached the top of the pole!!!");
//                }
//                else {
//                    System.out.println("Percent Color: " + percentColor);
//                }

                level2Assignment = false;
                level1Assigment = false;
                level3Assignment = true;
            }


            /**
             * outside of the level bracket (runs regardless of level) below
             */





        Imgproc.rectangle(inputOriginal, focusRect, red, 2);

        focusRectWidth = focusRect.width;
        focusRectHeight = focusRect.height;

            //For the moment, let's assume that the min width is 2 boxes, and min height 10 boxes

            //This will act as the red/green light indicator to the driver (MUST be correct)
            //If correct width of largest object, correct height, ALSO, isn't too big.
            if(focusRect != null && focusRectWidth >= minimumWidth && focusRectHeight >= minimumHeight){
                level2Capable = true;
            }else{
                level2Capable = false;
            }



//        inputMask.release();
//        inputMaskOriginal.release();
//        inputHSV.release();
//        input.release(); <-- Causes app exit/crash
//        inputOriginal.release(); <-- Causes app exit/crash


        if(levelString.equals("three")){
            return inputMask;
        }else{
            return inputOriginal;

        }
    }


    public static double getPercentColor(){
        return percentColor;
    }
    public static String getLevelString(){
        return levelString;
    }
    public static boolean getLevel2Capable(){
        return level2Capable;
    }

    public static int getXResolution(){
        return x_resolution;
    }

    public static int getYResolution(){
        return y_resolution;
    }

    public static int getRectWidth(){
        return focusRectWidth;
    }

    public static int getRectHeight(){
        return focusRectHeight;
    }

    public static int getMinRectWidth(){
        return minimumWidth;
    }

    public static int getMinRectHeight(){
        return minimumHeight;
    }

    public static int getBoxWidth(){
        return box_width;
    }

    public static int getBoxHeight(){
        return box_height;
    }

    public static int getLowestX(){
        return lowestX;
    }

    public static int getHighestX(){
        return highestX;
    }

    public static int getLowestY(){
        return lowestY;
    }

    public static int getHighestY(){
        return highestY;
    }

    public static int getBoxBL_X(){return boxBL_x;}

    public static int getBoxBL_Y(){
        return boxBL_y;
    }

    public static boolean getLevel1Assigment(){
        return level1Assigment;
    }

    public static boolean getLevel2Assigment(){
        return level2Assignment;
    }
    public static boolean getLevel3Assigment(){
        return level3Assignment;
    }


    public static int getCenterX(){
        return centerX;
    }

    public static int getLargestSize(){
        return largestSize;
    }

    public static int getLargestObjectWidth(){
        return largestObjectWidth;
    }


}
