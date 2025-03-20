package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class CL1ToHome extends SequentialCommandGroup {

  public CL1ToHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateHOME(),
      new PivotHomeCmd(pivot),
      new ExtenderHomeCmd(extender),
      new ElevatorHomeCmd(elevator)
    );
  }
}