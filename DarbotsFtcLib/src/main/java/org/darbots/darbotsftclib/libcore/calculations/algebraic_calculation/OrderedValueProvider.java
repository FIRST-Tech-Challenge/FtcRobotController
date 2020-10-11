package org.darbots.darbotsftclib.libcore.calculations.algebraic_calculation;

public interface OrderedValueProvider {
    boolean orderIncremental();
    double valueAt(double independentVar);
}
