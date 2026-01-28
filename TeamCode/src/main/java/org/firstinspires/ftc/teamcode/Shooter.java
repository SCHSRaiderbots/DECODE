package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter extends ShooterBase {
    // the shooter motor
    DcMotorEx motor;
    double ticksPerRev;
    double ticksPerSecondMax;

    Servo servo;

    double REST = 0.3;
    double PUSH = 0.1;

    public Shooter(HardwareMap hardwareMap) {
        // get the motor
        motor = hardwareMap.get(DcMotorEx.class, "motorShoot");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // set the gearing
        motor.getMotorType().setGearing(1.0);

        ticksPerRev = motor.getMotorType().getTicksPerRev();
        // was 560, probably gear ratio of 20 and 28 ticks per rev
        ticksPerRev = 28;

        // this is 2830. makes sense: 28 ticks/rev * 100 rev/sec
        ticksPerSecondMax = motor.getMotorType().getAchieveableMaxTicksPerSecond();

        // TODO: Set custom PIDF values
        double p = 200.0;
        double i = 0.0;
        double d = 0.0;

        // double f = 13.5;
        // expect to be about 32000 / 100 * 28 approx 10
        double f = 32000.0 / ticksPerSecondMax;
        // tweak setpoint for drag
        f = f * 2.1;
        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f, MotorControlAlgorithm.PIDF);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        motor.setPower(1.0);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor.setVelocity(0.0);

        servo = hardwareMap.get(Servo.class, "leverServo");

        servo.setPosition(REST);
    }

    /**
     * Set the shooter velocity.
     * @param rps speed in revolutions per second
     */
    public void setRPS(double rps) {
        // chose a decent speed to set the speed.
        // the native units are ticks per second
        motor.setVelocity(-rps * ticksPerRev);
    }

    /**
     * Get the shooter speed.
     * @return speed in revolutions per second
     */
    public double getRPS() {
        return motor.getVelocity() / ticksPerRev;
    }

    /**
     * Set the shooter velocity
     * @param mps velocity in meters per second
     */
    public void setMPS(double mps) {
        double rps = mps / ((125.0 / 60.0) * Math.PI * (2.0 * 0.0254));
        setRPS(rps);
    }

    /**
     * Get the shooter velocity
     * @return velocity in meters per second
     */
    public double getMPS() {
        //                gear ratio       circumference of wheel
        return getRPS() * (125.0 / 60.0) * Math.PI * (2.0 * 0.0254);
    }

    /** Feed Ball into the Shooter */
    public void feed() {
        servo.setPosition(PUSH);
    }

    /** Bring Spoon back*/
    public void back() {
        servo.setPosition(REST);
    }
}
