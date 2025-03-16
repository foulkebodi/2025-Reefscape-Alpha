package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.winch.WinchCmd;
import frc.robot.subsystems.WinchSys;

public class StateMachine {
    private WinchSys winch;
    
    public Command getCSG() {
        return new WinchCmd(winch);
    }
}
