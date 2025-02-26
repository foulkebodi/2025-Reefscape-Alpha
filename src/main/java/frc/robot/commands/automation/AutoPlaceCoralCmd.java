package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.coral.CoralOuttakeCmd;
import frc.robot.commands.coral.CoralStopCmd;
import frc.robot.subsystems.CoralSys;

public class AutoPlaceCoralCmd extends SequentialCommandGroup{
    public AutoPlaceCoralCmd(CoralSys coral) {
        new CoralOuttakeCmd(coral);
        new WaitCommand(0.5);
        new CoralStopCmd(coral);
    }
}