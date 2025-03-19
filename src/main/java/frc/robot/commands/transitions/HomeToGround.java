package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToGround extends SequentialCommandGroup {

  public HomeToGround(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ExtenderHomeCmd(extender),
      new ElevatorHomeCmd(elevator),
      new PivotGroundCmd(pivot)
    );
  }
}