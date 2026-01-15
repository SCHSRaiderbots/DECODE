package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Small Servo Move", group = "Test")
public class SmallServoMove extends LinearOpMode {

    private Servo servo;

    @Override
    public void runOpMode() {
        // Replace "servo" with the name in the Robot Configuration
        servo = hardwareMap.get(Servo.class, "servo");

        // Starting position
        servo.setPosition(0.5);

        waitForStart();

        if (opModeIsActive()) {
            // Move the servo a little bit
            servo.setPosition(0.6);

            // Wait so you can see the movement
            sleep(1000);

            // Optional: move it back
            servo.setPosition(0.5);
        }
    }
}
