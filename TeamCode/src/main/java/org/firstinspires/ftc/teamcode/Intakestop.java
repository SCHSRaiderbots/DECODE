package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class Intakestop extends Command {

    Intake m_intake;


    public Intakestop(Intake intake) {
        m_intake = intake;

    }
    @Override
    public void initialize()
    {


        m_intake.power(0.0);




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
