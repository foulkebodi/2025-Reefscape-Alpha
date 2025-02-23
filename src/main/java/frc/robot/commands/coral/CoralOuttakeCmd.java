package frc.robot.commands.coral;

import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSys;
import edu.wpi.first.wpilibj2.command.Command;

public class CoralOuttakeCmd extends Command {

    private final CoralSys coral;   

    public CoralOuttakeCmd(CoralSys coral){
        this.coral = coral;

        addRequirements(coral);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        coral.settargetRPM(CoralConstants.outtakeRPM);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 