package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCL3Cmd;
import frc.robot.commands.extender.ExtenderCL2Cmd;
import frc.robot.commands.pivot.PivotCL23PrepCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToCL3 extends SequentialCommandGroup {

  public HomeToCL3(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      // new StateCL3(),
      new ElevatorCL3Cmd(elevator),
      new ExtenderCL2Cmd(extender),
      new WaitCommand(0.2),
      new PivotCL23PrepCmd(pivot)
    );
  }
}