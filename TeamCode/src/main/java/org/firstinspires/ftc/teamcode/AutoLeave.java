package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name="AutoLeave", group = "competition")

public class AutoLeave extends OpMode {

    double v;
    long startTime;

    @Override
    public void init() {
        // initialize drivetrain
        Motion.robot = RobotId.identifyRobot(hardwareMap);
        Motion.init(hardwareMap);

        // same velocity math as your TeleOp
        double rpm = 6000.0;
        v = (rpm / 60.0) * Motion.HD_HEX_TICKS_PER_REV;
    }

    @Override
    public void start() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {

        long elapsed = System.currentTimeMillis() - startTime;

        // drive forward for 2 seconds
        if (elapsed < 2000) {
            Motion.setVelocity(v * 0.6, v * 0.6);
        }
        else {
            Motion.setVelocity(0, 0); // stop
        }
    }

    @Override
    public void stop() {
        Motion.setVelocity(0, 0);
    }
}
