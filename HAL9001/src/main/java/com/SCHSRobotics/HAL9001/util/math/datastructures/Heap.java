package com.SCHSRobotics.HAL9001.util.math.datastructures;

import org.jetbrains.annotations.Nullable;

import java.util.ArrayList;
import java.util.List;

/**
 * An abstract heap class, implementing the base functions of  a heap.
 * <p>
 * Creation Date: 5/17/20
 *
 * @param <T> The heap's datatype.
 * @author Cole Savage, Level Up
 * @version 1.0.0
 * @see MinHeap
 * @see MaxHeap
 * @since 1.1.0
 */
public abstract class Heap<T> {
    //A list storing the elements of the heap (plus null for the head of the list).
    protected List<T> heapList = new ArrayList<>();
    //The number of elements in the heap.
    protected int count = 0;

    /**
     * A constructor for heap.
     */
    public Heap() {
        heapList.add(null);
    }

    /**
     * Heapifys up after adding an element to restore the heap structure.
     *
     * @see MinHeap
     * @see MaxHeap
     */
    protected abstract void heapifyUp();

    /**
     * Heapifys down after removing an element to restore the heap structure.
     *
     * @see MinHeap
     * @see MaxHeap
     */
    protected abstract void heapifyDown();

    /**
     * Adds an element to the heap.
     *
     * @param element The element to add.
     */
    public final void add(T element) {
        count++;
        heapList.add(element);
        heapifyUp();
    }

    /**
     * Gets the value at the top of the heap, removes it, and then returns it.
     *
     * @return The value at the top of the heap.
     */
    @Nullable
    public final T poll() {
        if (count == 0) return null;

        T val = heapList.get(1);
        heapList.set(1, heapList.get(count));
        heapList.remove(count);
        count--;
        heapifyDown();
        return val;
    }

    /**
     * Gets the index of the parent element of a heap element given its index.
     *
     * @param idx The index of a heap element.
     * @return The index of its parent.
     */
    protected final int parentIdx(int idx) {
        return idx / 2;
    }

    /**
     * Gets the index of a heap element's left child index.
     *
     * @param idx The index of the heap element.
     * @return The index of the element's left child.
     */
    protected final int leftChildIdx(int idx) {
        return idx * 2;
    }

    /**
     * Gets the index of a heap element's right child index.
     *
     * @param idx The index of the heap element.
     * @return The index of the element's right child.
     */
    protected final int rightChildIdx(int idx) {
        return idx * 2 + 1;
    }

    /**
     * Returns whether a heap element has a child element.
     *
     * @param idx The index of the heap element.
     * @return Whether a heap element has a child element.
     */
    protected final boolean childPresent(int idx) {
        return leftChildIdx(idx) <= count;
    }

    /**
     * Gets the size of the heap.
     *
     * @return The size of the heap.
     */
    public final int size() {
        return count;
    }
}