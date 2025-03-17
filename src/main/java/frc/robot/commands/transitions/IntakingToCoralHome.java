package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.pivot.PivotCoralHomeCmd;
import frc.robot.subsystems.PivotSys;

public class IntakingToCoralHome extends SequentialCommandGroup {

  public IntakingToCoralHome(PivotSys pivot) {
    super(
      new PivotCoralHomeCmd(pivot)
    );
  }
}