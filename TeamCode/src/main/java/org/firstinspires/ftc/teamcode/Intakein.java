package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.command.Command;

public class Intakein extends Command {

    Intake m_intake;


    public Intakein(Intake intake) {
        m_intake = intake;

    }
    @Override
    public void initialize()
    {


        m_intake.power(1.0);




    }

    @Override
    public boolean isFinished() {

        return true;
    }

}
