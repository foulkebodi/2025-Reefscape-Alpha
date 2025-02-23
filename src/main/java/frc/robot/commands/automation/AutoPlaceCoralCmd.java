package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCoralThreeCmd;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.subsystems.ElevatorSys;

public class AutoPlaceCoralCmd extends SequentialCommandGroup{
    public AutoPlaceCoralCmd(ElevatorSys elevator) {
        new ElevatorCoralThreeCmd(elevator);
        new WaitCommand(1.0);
        new ElevatorHomeCmd(elevator);
    }
}
