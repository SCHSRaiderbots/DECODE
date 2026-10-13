package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Test Intake", group="Competition")
public class TestIntake extends OpMode {
      DcMotorEx intake;

    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void start() {
        intake.setPower(0.4);
    }

    public void loop() {

    }

    public void stop() {
        intake.setPower(0);
    }
}
