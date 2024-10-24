package org.nknsd.robotics.framework;

import java.util.HashMap;

public interface NKNAutoStep {
    void link(HashMap<String, NKNComponent> componentMap);
    void run();
    boolean isDone();
    String getName();
}
