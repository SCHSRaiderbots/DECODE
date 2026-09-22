package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Mecanum", group ="Competition")
@SuppressWarnings("unused")
public class Mecanum extends OpMode {
    DcMotorEx lfront;
    DcMotorEx rfront;
    DcMotorEx lrear;
    DcMotorEx rrear;

    @Override
    public void init() {
        lfront = hardwareMap.get(DcMotorEx.class, "lfront");
        rfront = hardwareMap.get(DcMotorEx.class, "rfront");
        lrear = hardwareMap.get(DcMotorEx.class, "lrear");
        rrear = hardwareMap.get(DcMotorEx.class, "rrear");

        rfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rrear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void start() {
        lfront.setPower(0);
        rfront.setPower(0);
        lrear.setPower(0);
        rrear.setPower(0);
    }

    @Override
    public void loop() {
        double lpower = -gamepad1.left_stick_y;
        double lrpower = -gamepad1.left_stick_x;
        double rrpower = -gamepad1.right_stick_x;

        lfront.setPower(lpower);
        rfront.setPower(lpower);
        rrear.setPower(lpower);
        lrear.setPower(lpower);

        lfront.setPower(-lrpower);
        rfront.setPower(lrpower);
        rrear.setPower(lrpower);
        lrear.setPower(-lrpower);

        lfront.setPower(-rrpower);
        rfront.setPower(rrpower);
        rrear.setPower(-rrpower);
        lrear.setPower(rrpower);

    }

    @Override
    public void stop() {
        lfront.setPower(0);
        rfront.setPower(0);
        lrear.setPower(0);
        rrear.setPower(0);
    }

}
