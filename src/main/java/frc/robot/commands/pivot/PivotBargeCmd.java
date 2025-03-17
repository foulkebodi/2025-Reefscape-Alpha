package frc.robot.commands.pivot;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotBargeCmd extends Command {

    private final PivotSys pivot;   

    public PivotBargeCmd(PivotSys pivot){
        this.pivot = pivot;

        addRequirements(pivot);
    }

    @Override
    public void initialize() {
        pivot.setTargetDeg(PivotConstants.bargeDeg);
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
 