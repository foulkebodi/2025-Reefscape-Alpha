package frc.robot.commands.extender;

import frc.robot.subsystems.ExtenderSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtenderHomeCmd extends Command {

private final ExtenderSys extender;   

public ExtenderHomeCmd(ExtenderSys extender){
    this.extender = extender;

    addRequirements(extender);
}

@Override
public void initialize() {
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
 