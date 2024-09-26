package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hangarm;

public class HangarmDown extends Command {
	Hangarm m_hangarm;

    public HangarmDown(Hangarm hangarm) {
        m_hangarm = hangarm;
        addRequirements(m_hangarm);
    }

    @Override
    public void execute() {
        m_hangarm.retract();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_hangarm.stop();
    }
}
