package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotCL1Cmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToCL1 extends SequentialCommandGroup {

  public HomeToCL1(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new PivotCL1Cmd(pivot),
      new ElevatorHomeCmd(elevator),
      new ExtenderHomeCmd(extender)
    );
  }
}