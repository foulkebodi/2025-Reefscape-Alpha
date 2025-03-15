package frc.robot.commands.climber;

import frc.robot.subsystems.ClimberSys;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class WinchManualCmd extends Command {
    
    
    private final ClimberSys climber;
    
    private final DoubleSupplier input;

    public WinchManualCmd(DoubleSupplier input, ClimberSys climber){
        this.climber = climber;
        this.input = input;

        addRequirements(climber);
    }
@Override 
public void initialize(){}

@Override 
public void execute(){
    climber.winchManual(Math.abs(input.getAsDouble() * 0.5));
}
@Override
public void end(boolean interrupted){}
@Override
public boolean isFinished(){
    return false;
}
}