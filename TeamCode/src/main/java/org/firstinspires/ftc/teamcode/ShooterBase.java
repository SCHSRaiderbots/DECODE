package org.firstinspires.ftc.teamcode;

public class ShooterBase {

    public ShooterBase() {
        // do nothing
    }

    /**
     * Set the shooter velocity.
     * @param rps speed in rotations per second
     */
    public void setRPS(double rps) {
        // do nothing
    }

    public double getRPS() {
        // return bogus value
        return -1.0;
    }

    public void setMPS(double mps) {
        double rps = mps / ((125.0 / 60.0) * Math.PI * (2.0 * 0.0254));
        setRPS(rps);
    }

    public double getMPS() {
        //                gear ratio       circumference of wheel
        return getRPS() * (125.0 / 60.0) * Math.PI * (2.0 * 0.0254);
    }

    /** Feed Ball into the Shooter */
    public void feed() {
        // do nothing
    }

    /** Bring Spoon back*/
    public void back() {
        // do nothing
    }
}
