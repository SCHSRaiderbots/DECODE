package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
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

        // Optional: Set custom PIDF values
        double p = 20.0;
        double i = 1.0;
        double d = 0.0;

        // double f = 13.5;
        // expect to be about 32000 / 100 * 28 approx 10
        double f = 32000.0 / ticksPerSecondMax;
        f = f * 1.8;
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
     * @param rps
     */
    public void setRPM(double rps) {
        // chose a decent speed to set the speed.
        // the native units are ticks per second
        motor.setVelocity(rps * ticksPerRev);
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
