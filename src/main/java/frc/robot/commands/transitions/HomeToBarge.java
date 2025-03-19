package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorBargeCmd;
import frc.robot.commands.extender.ExtenderBargeCmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToBarge extends SequentialCommandGroup {

  public HomeToBarge(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorBargeCmd(elevator),
      new PivotHomeCmd(pivot),
      new WaitCommand(0.2),
      new ExtenderBargeCmd(extender)
    );
  }
}