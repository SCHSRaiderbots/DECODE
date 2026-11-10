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

    TestIntake testIntake;
    TestIMU testIMU;


    @Override
    public void init() {
        lfront = hardwareMap.get(DcMotorEx.class, "lfront");
        rfront = hardwareMap.get(DcMotorEx.class, "rfront");
        lrear = hardwareMap.get(DcMotorEx.class, "lrear");
        rrear = hardwareMap.get(DcMotorEx.class, "rrear");

        testIntake = new TestIntake(hardwareMap);

        rfront.setDirection(DcMotorSimple.Direction.REVERSE);
        rrear.setDirection(DcMotorSimple.Direction.REVERSE);

        try {
            testIMU = new TestIMU(hardwareMap);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
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
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", testIMU.getAngle());

        // reset Yaw
        if (gamepad1.aWasPressed())
        {
            telemetry.addData("Yaw", "Resetting\n");
            testIMU.resetYaw();
        }

        //lpower is the control for forward and backward on the left stick
        double lpower = -gamepad1.left_stick_y;
        //lrpower is the control for rotate c and cc on the left stick
        double rrpower = -gamepad1.left_stick_x;
        //rrpower is the control for left and right on the right stick
        double lrpower = -gamepad1.right_stick_x;

        double yawAngle = testIMU.getAngle();
        /*
        // change to field-centric
        rrpower = rrpower * Math.cos(-yawAngle) + lpower * Math.sin(-yawAngle);
        lpower = lpower * Math.cos(-yawAngle) - rrpower * Math.sin(-yawAngle);
        */
        // powers to the motors
        double pwrLFront = lpower;
        double pwrRFront = lpower;
        double pwrLRear = lpower;
        double pwrRRear = lpower;

        // rotate
        pwrLFront += -lrpower;
        pwrRFront += lrpower;
        pwrRRear += lrpower;
        pwrLRear += -lrpower;


        // left/right
        pwrLFront += -rrpower;
        pwrRFront += rrpower;
        pwrRRear += -rrpower;
        pwrLRear += rrpower;

        // find the max absolute power
        double maxpwr = Math.max(Math.abs(pwrLFront), Math.abs(pwrRFront));
        maxpwr =  Math.max(maxpwr, Math.abs(pwrLRear));
        maxpwr =  Math.max(maxpwr, Math.abs(pwrRRear));

        double scale = (maxpwr > 1.0) ? 1.0 / maxpwr : 1.0;

        // after computing all the powers, set the motors to their power
        lfront.setPower(scale * pwrLFront * 0.7);
        rfront.setPower(scale * pwrRFront * 0.7);
        lrear.setPower(scale * pwrLRear * 0.7);
        rrear.setPower(scale * pwrRRear * 0.7);

        // controls for intake
        double intake = gamepad1.right_trigger;
        double eject = -1 * gamepad1.left_trigger;

        // intake power
        double pwrIntake = 0;
        pwrIntake += intake;
        pwrIntake += eject;

        testIntake.setPower(pwrIntake);
    }

    @Override
    public void stop() {
        lfront.setPower(0);
        rfront.setPower(0);
        lrear.setPower(0);
        rrear.setPower(0);

        testIntake.setPower(0.0);
    }

}
