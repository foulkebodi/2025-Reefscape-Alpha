package frc.robot.commands.coral;

import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCmd extends Command {

    private final IntakeSys coral;   

    public CoralOuttakeCmd(IntakeSys coral){
        this.coral = coral;

        addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setIsOuttaking(true);
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
 