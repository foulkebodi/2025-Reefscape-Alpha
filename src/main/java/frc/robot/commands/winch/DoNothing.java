package frc.robot.commands.winch;

import frc.robot.Constants.WinchConstants;
import frc.robot.subsystems.WinchSys;
import edu.wpi.first.wpilibj2.command.Command;

public class DoNothing extends Command {

    private final WinchSys winch;   

    public DoNothing(WinchSys winch) {
        this.winch = winch;

        addRequirements(winch);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override 
    public boolean isFinished() {
        return true;
    }
}
 