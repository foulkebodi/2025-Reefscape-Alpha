package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotProcessorCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class AlgaeHomeToProcessor extends SequentialCommandGroup {

  public AlgaeHomeToProcessor(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ExtenderHomeCmd(extender),
      new ElevatorHomeCmd(elevator),
      new PivotProcessorCmd(pivot)
    );
  }
}