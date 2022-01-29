package com.SCHSRobotics.HAL9001.util.misc;

import com.SCHSRobotics.HAL9001.util.exceptions.DumpsterFireException;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * A class used to perform non-max suppression on a list of bounding boxes.
 * The algorithm is essentially a port of the Malisiewicz et al. method described in http://www.computervisionblog.com/2011/08/blazing-fast-nmsm-from-exemplar-svm.html.
 * <p>
 * Creation Date: 1/15/19
 *
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @since 1.0.0
 */
@SuppressWarnings("unused")
public class NonMaxSuppressor {
    //Todo move helper functions to FakeNumpy
    //The area overlap threshold.
    private double thresh;

    /**
     * Constructor for NonMaxSuppressor.
     *
     * @param thresh The area overlap threshold.
     */
    @Contract(pure = true)
    public NonMaxSuppressor(double thresh) {
        this.thresh = thresh;
    }

    /**
     * Performs non-max suppression.
     *
     * @param boxes A list of bounding boxes.
     * @return A list of merged bounding boxes.
     */
    public List<Rect> suppressNonMax(@NotNull List<Rect> boxes) {
        if(boxes.size() == 0) {
            return boxes;
        }

        List<Integer> selectedIdxes = new ArrayList<>();

        List<Double> x1 = new ArrayList<>(); //top left x values.
        List<Double> y1 = new ArrayList<>(); //top left y values.
        List<Double> x2 = new ArrayList<>(); //bottom right x values.
        List<Double> y2 = new ArrayList<>(); //bottom right y values.

        List<Double> areas = new ArrayList<>();

        for(Rect box : boxes) {

            //Adds box corners to their respective lists.
            x1.add((double) box.x);
            y1.add((double) box.y);
            x2.add((double) box.x+box.width);
            y2.add((double) box.y+box.height);

            //Calculates area of the bounding box. the + 1 is included so that a bounding box with 0 area can't exist.
            areas.add((x2.get(boxes.indexOf(box))-x1.get(boxes.indexOf(box))+1)*(y2.get(boxes.indexOf(box))-y1.get(boxes.indexOf(box))+1));
        }

        //Sort the boxes by bottom right y coordinate and store their indexes.
        List<Integer> idxes = argsort(y2);

        while(idxes.size() > 0) {

            int last = idxes.size() - 1;
            int i = idxes.get(last);

            selectedIdxes.add(i);

            //Get the largest x and y coordinates for the start of the bounding boxes, and the smallest x and y coordinates for the end of the bounding boxes.
            List<Double> xx1 = listMax(x1.get(i),x1,idxes.subList(0,last));
            List<Double> yy1 = listMax(y1.get(i),y1,idxes.subList(0,last));
            List<Double> xx2 = listMin(x2.get(i),x2,idxes.subList(0,last));
            List<Double> yy2 = listMin(y2.get(i),y2,idxes.subList(0,last));

            //Get the width and height of the bounding boxes
            List<Double> w = listMax(0,addListConstant(subtractLists(xx2,xx1),1));
            List<Double> h = listMax(0,addListConstant(subtractLists(yy2,yy1),1));

            //Calculate the area overlap ratio of the bounding boxes
            List<Double> overlap = divideLists(multiplyLists(w,h),areas,idxes.subList(0,last));

            idxes.remove(last);

            //Delete all indexes when the area overlap ratio > threshold.
            deleteBadIndexes(idxes,overlap);

        }

        //Return a list of the best detected bounding boxes.
        return populateList(boxes,selectedIdxes);
    }

    /**
     * Adds all boxes that have references in idxes to a list, and returns that list
     *
     * @param boxes The original list of bounding boxes.
     * @param idxes The indexes of the bounding boxes to use to populate the list.
     * @return A list containing all the bounding boxes referenced by idxes.
     */
    @NotNull
    private List<Rect> populateList(@NotNull List<Rect> boxes, @NotNull List<Integer> idxes) {
        List<Rect> output = new ArrayList<>();
        for(int idx : idxes) {
            output.add(boxes.get(idx));
        }
        return output;
    }

    /**
     * Adds a constant value to every value in a list.
     *
     * @param lst The list to add a constant to.
     * @param x The constant to add.
     * @return The original list where every value has a constant added.
     */
    @NotNull
    @SuppressWarnings("SameParameterValue")
    private List<Double> addListConstant(@NotNull List<Double> lst, double x) {
        List<Double> output = new ArrayList<>();
        for(double val : lst) {
            output.add(val+x);
        }
        return output;
    }

    /**
     * Subtract the elements of two lists.
     *
     * @param lst1 The first list.
     * @param lst2 The second list.
     * @return list1 list2.
     */
    @NotNull
    private List<Double> subtractLists(@NotNull List<Double> lst1, @NotNull List<Double> lst2) {
        if(lst1.size() != lst2.size()) {
            throw new DumpsterFireException("Lists not the same size!");
        }
        List<Double> output = new ArrayList<>();
        for (int i = 0; i < lst1.size(); i++) {
            output.add(lst1.get(i)-lst2.get(i));
        }
        return output;
    }

    /**
     * Divide the elements of two lists.
     *
     * @param lst1 The first list.
     * @param lst2 The second list.
     * @param mask The indexes that should be divided.
     * @return list1/list2 for the indexes in mask.
     */
    @NotNull
    private List<Double> divideLists(@NotNull List<Double> lst1, @NotNull List<Double> lst2, @NotNull List<Integer> mask) {
        List<Double> output = new ArrayList<>();
        for (int i = 0; i < mask.size(); i++) {
            output.add(lst1.get(i)/lst2.get(mask.get(i)));
        }
        return output;
    }

    /**
     * Multiply the elements of two lists.
     *
     * @param lst1 The first list.
     * @param lst2 The second list.
     * @return list1 * list2.
     */
    @NotNull
    private List<Double> multiplyLists(@NotNull List<Double> lst1, @NotNull List<Double> lst2) {
        if(lst1.size() != lst2.size()) {
            throw new DumpsterFireException("Lists not the same size!");
        }

        List<Double> output = new ArrayList<>();
        for (int i = 0; i < lst1.size(); i++) {
            output.add(lst1.get(i)*lst2.get(i));
        }
        return output;
    }

    /**
     * Deletes all indexes where the corresponding overlap is greater than the overlap threshold.
     *
     * @param idxes The list of indexes of the bounding boxes.
     * @param overlaps The list of area overlap ratios for the bounding boxes.
     */
    private void deleteBadIndexes(@NotNull List<Integer> idxes, @NotNull List<Double> overlaps) {
        for(int i = 0; i < overlaps.size(); i++) {
            if(overlaps.get(i) > thresh ) {
                idxes.remove(i);
                break;
            }
        }
    }

    /**
     * Performs Math.max() on every element in the given list with a given constant.
     *
     * @param x The value being compared with every element in the list.
     * @param lst The list.
     * @param mask A list of valid indexes for the max() operation to take place.
     * @return A list where all elements < x are replaced with x.
     */
    @NotNull
    private List<Double> listMax(double x, @NotNull List<Double> lst, @NotNull List<Integer> mask) {
        List<Double> output = new ArrayList<>();
        for(int idx : mask) {
            output.add(Math.max(x,lst.get(idx)));
        }
        return output;
    }

    /**
     * Performs Math.max() on every element in the given list with a given constant.
     *
     * @param x The value being compared with every element in the list.
     * @param lst The list.
     * @return A list where all elements < x are replaced with x.
     */
    @NotNull
    @SuppressWarnings("SameParameterValue")
    private List<Double> listMax(double x, @NotNull List<Double> lst) {
        List<Double> output = new ArrayList<>();
        for(double val : lst) {
            output.add(Math.max(x,val));
        }
        return output;
    }

    /**
     * Performs Math.min() on every element in the given list with a given constant.
     *
     * @param x The value being compared with every element in the list.
     * @param lst The list.
     * @param mask A list of valid indexes for the min() operation to take place.
     * @return A list where all elements > x are replaced with x.
     */
    @NotNull
    private List<Double> listMin(double x, @NotNull List<Double> lst, @NotNull List<Integer> mask) {
        List<Double> output = new ArrayList<>();
        for(int idx : mask) {
            output.add(Math.min(x,lst.get(idx)));
        }
        return output;
    }

    /**
     * Sorts the input list and returns sorted list in the form of references to the input list.
     *
     * @param input The input list.
     * @return A list of the indexes of each of the elements in the sorted list (in the sorted order).
     */
    @NotNull
    private List<Integer> argsort(@NotNull List<Double> input) {
        List<Integer> output = new ArrayList<>();
        List<Double> cpy = new ArrayList<>(input);
        List<Double> lstSorted = new ArrayList<>(input);

        Collections.sort(lstSorted);

        for (double val : lstSorted) {
            output.add(cpy.indexOf(val));
            cpy.set(cpy.indexOf(val),-1.0);
        }
        return output;
    }
}