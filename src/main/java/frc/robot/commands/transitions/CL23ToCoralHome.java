package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotCoralHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL23ToCoralHome extends SequentialCommandGroup {

  public CL23ToCoralHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorHomeCmd(elevator),
      new WaitCommand(0.1),
      new PivotCoralHomeCmd(pivot),
      new ExtenderHomeCmd(extender)
    );
  }
}