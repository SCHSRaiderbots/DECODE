package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class ShooterCommand extends Command {
    double m_speed;
    Shooter m_shooter;

    public ShooterCommand(Shooter shooter, double speed) {


        m_shooter = shooter;
        m_speed = speed;
    }
    @Override
    public void initialize()
    {
        m_shooter.setRPS(m_speed);


        m_shooter.feed();

    }
    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {}
}