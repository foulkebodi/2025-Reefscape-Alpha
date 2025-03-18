package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorCL2Cmd;
import frc.robot.commands.extender.ExtenderALCmd;
import frc.robot.commands.pivot.PivotALCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class AlgaeHomeToAL2 extends SequentialCommandGroup {

  public AlgaeHomeToAL2(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorCL2Cmd(elevator),
      new WaitCommand(0.2),
      new ExtenderALCmd(extender),
      new PivotALCmd(pivot)
    );
  }
}