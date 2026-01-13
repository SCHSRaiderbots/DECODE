package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Shooter {
    // the shooter motor
    DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        // get the motor
        motor = hardwareMap.get(DcMotorEx.class, "motorShoot");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Optional: Set custom PIDF values
        PIDFCoefficients pidf = new PIDFCoefficients(
                10.0,
                3.0,
                0.0,
                13.5);

        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        motor.setPower(1.0);

        motor.setVelocity(0.0);

    }

    public void setRPM(double rps) {
        motor.setVelocity(rps, AngleUnit.RADIANS);
    }
}
