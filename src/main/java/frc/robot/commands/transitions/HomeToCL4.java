package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.StateCL4;
import frc.robot.commands.elevator.ElevatorBargeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToCL4 extends SequentialCommandGroup {

  public HomeToCL4(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateCL4(),
      new ElevatorBargeCmd(elevator),
      new WaitCommand(0.2),
      new ExtenderHomeCmd(extender),
      new PivotGroundCmd(pivot)
    );
  }
}