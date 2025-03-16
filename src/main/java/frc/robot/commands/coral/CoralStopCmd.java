package frc.robot.commands.coral;

import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralStopCmd extends Command {

    private final IntakeSys coral;   

    public CoralStopCmd(IntakeSys coral){
        this.coral = coral;

        addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setIsOuttaking(false);
    }

    @Override
    public void execute(){
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 