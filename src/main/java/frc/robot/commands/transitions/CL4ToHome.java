package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL4ToHome extends SequentialCommandGroup {

  public CL4ToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateHOME(),
      new PivotGroundCmd(pivot),
      new WaitCommand(0.2),
      new ElevatorHomeCmd(elevator),
      new WaitCommand(0.05),
      new PivotHomeCmd(pivot),
      new ExtenderHomeCmd(extender)
    );
  }
}