package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorBargeCmd;
import frc.robot.commands.extender.ExtenderBargeCmd;
import frc.robot.commands.pivot.PivotBargeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class AlgaeHomeToBarge extends SequentialCommandGroup {

  public AlgaeHomeToBarge(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorBargeCmd(elevator),
      new WaitCommand(0.5),
      new PivotBargeCmd(pivot),
      new ExtenderBargeCmd(extender)
    );
  }
}