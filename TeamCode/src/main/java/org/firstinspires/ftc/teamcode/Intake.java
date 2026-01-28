package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake extends IntakeBase {
    private final DcMotorEx motorIntake;

    public Intake(HardwareMap hardwareMap) {
        // get the motor
        motorIntake = hardwareMap.get(DcMotorEx.class, "motorIntake");

        // tell me about the motor
        LogDevice.dump("Intake Motor", motorIntake);

        power(0.0);
    }

    public void power(double power) {
        motorIntake.setPower(power);
    }
}
