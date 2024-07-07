package org.firstinspires.ftc.teamcode.common.commands.armCommands;
import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.depositSubsystem;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

public class Hang extends InstantCommand{
    public Hang(){
        super(() -> robotHardware.getInstance().intake.updateState(intakeSubsystem.intakeState.hang));
    }
}