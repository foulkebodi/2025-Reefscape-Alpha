package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class AL3ToHome extends SequentialCommandGroup {

  public AL3ToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ExtenderHomeCmd(extender),
      new PivotHomeCmd(pivot),
      new WaitCommand(0.3),
      new ElevatorHomeCmd(elevator)
    );
  }
}