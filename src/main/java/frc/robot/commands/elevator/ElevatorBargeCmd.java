package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSys;

public class ElevatorBargeCmd extends Command {

    private final ElevatorSys elevator;   

    public ElevatorBargeCmd(ElevatorSys elevator){
        this.elevator = elevator;

        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        elevator.setTargetInches(ElevatorConstants.bargeInches);
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
