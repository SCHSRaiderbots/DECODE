package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "Motor: Shooter", group = "Motor")
public class ShooterTest extends LinearOpMode {
    // Shooter Motor
    DcMotorEx motor;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "motorShoot");

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        while (opModeIsActive()) {
            motor.setPower(1.0);
        }

        motor.setPower(0.0);
    }

}
