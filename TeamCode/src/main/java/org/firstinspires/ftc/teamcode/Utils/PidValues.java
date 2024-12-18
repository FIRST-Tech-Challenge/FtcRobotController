package org.firstinspires.ftc.teamcode.Utils;

import java.util.Objects;

public class PidValues {
    double p, i, d, maxIOutput;

    public PidValues(PidValues pidValues){
        this.p = pidValues.p;
        this.i = pidValues.i;
        this.d = pidValues.d;
        this.maxIOutput = pidValues.maxIOutput;
    }

    public PidValues(double p, double i, double d, double maxIOutput, String pidName) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.maxIOutput = maxIOutput;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        PidValues pidValues = (PidValues) o;
        return Double.compare(pidValues.p, p) == 0 && Double.compare(pidValues.i, i) == 0 && Double.compare(pidValues.d, d) == 0 && Double.compare(pidValues.maxIOutput, maxIOutput) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(p, i, d, maxIOutput);
    }
}
