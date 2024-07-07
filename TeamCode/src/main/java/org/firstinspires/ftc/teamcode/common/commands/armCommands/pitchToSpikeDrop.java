package org.firstinspires.ftc.teamcode.common.commands.armCommands;


import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import com.arcrobotics.ftclib.command.InstantCommand;

public class pitchToSpikeDrop extends InstantCommand{
    public pitchToSpikeDrop(){
        super(() -> robotHardware.getInstance().deposit.updatePitchState(depositSubsystem.armState.spikedrop));
    }
}
