package com.SCHSRobotics.HAL9001.util.math.datastructures;

/**
 * An implementation of a max heap data structure.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <T> The datatype of the heap structure. Must implement comparable interface.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see Heap
 * @see MinHeap
 * @see Comparable
 * @since 1.1.0
 */
public class MaxHeap<T extends Comparable<T>> extends Heap<T> {

    @Override
    protected void heapifyUp() {
        int idx = count;
        int parent = parentIdx(idx);
        while (parent > 0) {
            if (heapList.get(parent).compareTo(heapList.get(idx)) < 0) {
                T temp = heapList.get(parent);
                heapList.set(parent, heapList.get(idx));
                heapList.set(idx, temp);
            }
            idx = parent;
            parent = parentIdx(idx);
        }
    }

    @Override
    protected void heapifyDown() {
        int idx = 1;
        while(childPresent(idx)) {
            int largerChildIdx = getLargerChild(idx);
            if (heapList.get(idx).compareTo(heapList.get(largerChildIdx)) < 0) {
                T temp = heapList.get(largerChildIdx);
                heapList.set(largerChildIdx, heapList.get(idx));
                heapList.set(idx, temp);
            }
            idx = largerChildIdx;
        }
    }

    /**
     * Gets which of the children of a given heap element is larger.
     *
     * @param idx The index of the heap element.
     * @return The index of the larger child element.
     */
    private int getLargerChild(int idx) {
        int leftIdx = leftChildIdx(idx);
        int rightIdx = rightChildIdx(idx);
        if (rightIdx > count) {
            return leftIdx;
        }
        T leftChild = heapList.get(leftIdx);
        T rightChild = heapList.get(rightIdx);

        //negative means less than, 0 means equal, positive means greater than
        if (leftChild.compareTo(rightChild) > 0) return leftIdx;
        return rightIdx;
    }
}