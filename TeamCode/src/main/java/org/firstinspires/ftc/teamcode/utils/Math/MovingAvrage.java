package org.firstinspires.ftc.teamcode.utils.Math;


import java.util.ArrayList;


public class MovingAvrage {

    ArrayList<Double> m_buffer;
    int m_windowSize;
    public MovingAvrage(int windowSize){
        m_windowSize=windowSize;
        m_buffer=new ArrayList<>(m_windowSize);
        for (int i = 0; i < windowSize; i++) {
            m_buffer.add(0.0);
        }
    }
    public double calculate(double measurement){
        m_buffer.remove(m_windowSize-1);
        m_buffer.add(0,measurement);
        double sum=0;
        for (int i = 0; i < m_windowSize; i++) {
            sum+=m_buffer.get(i);
        }
        return sum/m_windowSize;
    }
}