package org.nknsd.robotics.team;

import org.nknsd.robotics.framework.NKNComponent;
import org.nknsd.robotics.framework.NKNProgram;

import java.util.List;

public class SampleNKNProgram extends NKNProgram {
    @Override
    public void createComponents(List<NKNComponent> components) {
        SampleNKNComponent sample = new SampleNKNComponent();
        components.add(sample);
    }
}
