package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.ParallelCommandGroup;
import org.firstinspires.ftc.teamcode.command.SequentialCommandGroup;


@Autonomous(name="AutoMoveShoot", group="competition")
public class AutoMoveShoot extends OpMode {
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

        Motion.setPoseInches(44, -60, 135.0);


        command = new SequentialCommandGroup(
                // .. several

                new DriveTurnTowards(0, 0),
                new DriveForward(20),
                new ShooterCommand(shooter, 23)




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
        if(!command.isFinished() )
        {
            command.execute();
        }







    }









}
