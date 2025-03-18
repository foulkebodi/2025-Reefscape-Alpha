package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCL4Cmd;
import frc.robot.commands.extender.ExtenderCL4Cmd;
import frc.robot.commands.pivot.PivotCL4Cmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CoralHomeToCL4 extends SequentialCommandGroup {

  public CoralHomeToCL4(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorCL4Cmd(elevator),
      new WaitCommand(0.2),
      new ExtenderCL4Cmd(extender),
      new PivotCL4Cmd(pivot)
    );
  }
}