package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class ShooterSpin extends Command {
    double m_speed;
    Shooter m_shooter;
    double waittime;

    public ShooterSpin(Shooter shooter, double speed) {


        m_shooter = shooter;
        m_speed = speed;
        waittime = 4.5;
    }
    @Override
    public void initialize()
    {
        m_shooter.setRPS(m_speed);




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