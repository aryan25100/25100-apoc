package org.firstinspires.ftc.teamcode.common.commands.intakeCommands;

import org.firstinspires.ftc.teamcode.common.robot.robotHardware;
import org.firstinspires.ftc.teamcode.common.subsystem.intakeSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

public class flapDownStationary extends InstantCommand{
    public flapDownStationary(){
        super(() -> robotHardware.getInstance().intake.updateState(intakeSubsystem.intakeState.hang));
    }
}