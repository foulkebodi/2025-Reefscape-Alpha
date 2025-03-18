package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotAlgaeHomeCmd;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class BargeToAlgaeHome extends SequentialCommandGroup {

  public BargeToAlgaeHome(PivotSys pivot, ElevatorSys elevator, ExtenderSys extender) {
    super(
      new PivotAlgaeHomeCmd(pivot),
      new ExtenderHomeCmd(extender),
      new WaitCommand(0.25),
      new ElevatorHomeCmd(elevator)
    );
  }
}