package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.StateCL2;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderCL2Cmd;
import frc.robot.commands.pivot.PivotCL23PrepCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToCL2 extends SequentialCommandGroup {

  public HomeToCL2(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateCL2(),
      new ElevatorHomeCmd(elevator),
      new ExtenderCL2Cmd(extender),
      new WaitCommand(0.05),
      new PivotCL23PrepCmd(pivot)
    );
  }
}