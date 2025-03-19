package frc.robot.commands.transitions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.pivot.PivotCoralHomeCmd;
import frc.robot.commands.winch.DoNothing;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.WinchSys;

public class DoNothings extends SequentialCommandGroup {
  public DoNothings(WinchSys winch) {
    super(
      new DoNothing(winch)
    );
  }
}