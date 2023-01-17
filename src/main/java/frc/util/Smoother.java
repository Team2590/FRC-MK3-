package frc.util;

/**
 * Smooths values over time using averages
 * 
 * @author Elan Ronen
 */
public class Smoother {

    public final int size;      // Number of measurements to store
    private double[] values;    // Queue
    private double sum;         // Sum of numbers in the queue
    private int sumN;           // Number of items in the queue
    private int i = -1;         // Index of the most recent item
    
    /**
     * The constructor
     * 
     * @param n - the number of values to remember
     */
    public Smoother(int n) {
        size = n;
        values = new double[size];
    }

    /**
     * Add a value (and delete the oldest if more than n values)
     * 
     * @param v - the value to add
     * @return the average
     */
    public double push(double v) {
        i = (i + 1) % size;
        sum -= values[i];
        sum += v;
        values[i] = v;

        if (sumN != size) sumN++;

        return sum / sumN;
    }

    /**
     * Get the average
     * 
     * @return the average
     */
    public double get() {
        return sum / sumN;
    }

    /**
     * Reset to initial state
     */
    public void reset() {
        for (int i = 0; i < size; i++) {
            values[i] = 0;
        }
        sum = 0;
        sumN = 0;
        i = -1;
    }
}