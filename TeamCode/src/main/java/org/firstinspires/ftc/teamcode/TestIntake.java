package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestIntake {
    DcMotorEx intake;

    public TestIntake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setPower(double power) {
        // set the intake to that power
        intake.setPower(power);
    }

}
