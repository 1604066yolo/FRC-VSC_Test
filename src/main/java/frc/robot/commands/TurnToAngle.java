/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class TurnToAngle extends Command {

    private final double angle;
    
    private boolean isFinished;
    private boolean inTolerance;
    private int clockCycles;

    public TurnToAngle(final double angle) {
        this.angle = angle;

        requires(Robot.m_drivetrain);
    }

    // Called just before this Command runs the first time
    @Override
    protected void initialize() {
        Robot.m_drivetrain.rotateDegrees(angle);
    }

    // Called repeatedly when this Command is scheduled to run
    @Override
    protected void execute() {
        double error = Robot.m_drivetrain.turnController.getError();
        inTolerance = Math.abs(error) < 2;
        clockCycles = inTolerance ? clockCycles++ : 0;
        isFinished = clockCycles >= 5;
    }

    // Make this return true when this Command no longer needs to run execute()
    @Override
    protected boolean isFinished() {
        return isFinished;
    }

    // Called once after isFinished returns true
    @Override
    protected void end() {
        Robot.m_drivetrain.turnController.disable();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    @Override
    protected void interrupted() {
        end();
    }

}
