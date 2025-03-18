package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCL3Cmd;
import frc.robot.commands.extender.ExtenderCL23Cmd;
import frc.robot.commands.pivot.PivotCL23Cmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CoralHomeToCL3 extends SequentialCommandGroup {

  public CoralHomeToCL3(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorCL3Cmd(elevator),
      new WaitCommand(0.2),
      new ExtenderCL23Cmd(extender),
      new PivotCL23Cmd(pivot)
    );
  }
}