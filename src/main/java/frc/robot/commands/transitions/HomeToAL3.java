package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorAL2Cmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class HomeToAL3 extends SequentialCommandGroup {

  public HomeToAL3(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new ElevatorAL2Cmd(elevator),
      new WaitCommand(0.2),
      new PivotGroundCmd(pivot)
    );
  }
}