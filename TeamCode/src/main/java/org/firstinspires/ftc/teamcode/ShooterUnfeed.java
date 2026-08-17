package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class ShooterUnfeed extends Command {

    Shooter m_shooter;


    public ShooterUnfeed(Shooter shooter) {
        m_shooter = shooter;

    }
    @Override
    public void initialize()
    {


        m_shooter.back();




    }
    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {

        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}