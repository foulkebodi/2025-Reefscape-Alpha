package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotCL23Cmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL2ToHome extends SequentialCommandGroup {

  public CL2ToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateHOME(),
      new PivotCL23Cmd(pivot),
      new WaitCommand(0.2),
      new ExtenderHomeCmd(extender),
      new WaitCommand(0.1),
      new ElevatorHomeCmd(elevator),
      new PivotHomeCmd(pivot)
    );
  }
}