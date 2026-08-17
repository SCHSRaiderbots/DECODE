package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.command.SequentialCommandGroup;


@Autonomous(name="Good Auto", group="competition")
public class AutoLeave extends OpMode {
    Command command;
    Vision vision;

    Shooter shooter;
    Intake intake;




    //Add stuff that needs to be initiated
    public void init()
    {
        Motion.init(hardwareMap);

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        Motion.setPoseInches(-44, -60, 135.0);

        command = new SequentialCommandGroup(
                // .. several

                // spin shouter
                // spin shooter up,
                new ShooterSpin(shooter, 25),
                new DriveForward(-55.5),
                // delay
                new Delay(2.5),
                new ShooterFeed(shooter),
                new Delay(0.5),
                new ShooterUnfeed(shooter),
                new ShooterSpin(shooter, 0),
                new Delay(3),
                //Second Ball
                new ShooterSpin(shooter, 10),
                new Delay(1.0),
                new Intakein(intake),

                new Delay(1.5),

                new Intakestop(intake),
                new ShooterSpin(shooter, 0),




                new Delay(0.5),
                new ShooterSpin(shooter, 25),

                new Delay(2.5),

                new ShooterFeed(shooter),
                new Delay(0.5),
                new ShooterUnfeed(shooter),

                // spin down shooter
                new ShooterSpin(shooter, 0),
                new DriveTurnTowards(0, 134),

                new Delay(1.5),
                new DriveForward(15)

        );
    }

    public void init_loop() {
        // figure Blue/Red and starting position
    }

    //Program starts
    public void start() {
        command.initialize();
        command.execute();
    }

    public void loop() {
        if (!command.isFinished())
        {
            command.execute();
        }

    }

    public void stop() {
        command.cancel();
    }








}
