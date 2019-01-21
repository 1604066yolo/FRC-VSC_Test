/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.TankDrive;
import frc.robot.util.Util;

public class Drivetrain extends Subsystem {

    private static final WPI_TalonSRX leftMotor = new WPI_TalonSRX(RobotMap.drivetrain_leftMotor.value);
    private static final WPI_TalonSRX rightMotor = new WPI_TalonSRX(RobotMap.drivetrain_rightMotor.value);
    private static final WPI_TalonSRX leftFollower = new WPI_TalonSRX(RobotMap.drivetrain_leftMotorFollower.value);
    private static final WPI_TalonSRX rightFollower = new WPI_TalonSRX(RobotMap.drivetrain_rightMotorFollower.value);

    private static final TurnController turnController = new TurnController();

    private final DifferentialDrive drive;

    public Drivetrain() {
        Util.initTalon(leftMotor, rightMotor, leftFollower, rightFollower);
        leftFollower.follow(leftMotor);
        rightFollower.follow(rightMotor);

        drive = new DifferentialDrive(leftMotor, rightMotor);
    }

    /**
     * Turns the robot on its center to a specified angle
     * @param degrees the angle to turn to; positive for clockwise, negative for counter-clockwise
     */
    public void turnToAngle(double degrees) {
        turnController.turnToAngle(degrees);
    }

    public void stopMotors() {
        setMotors(ControlMode.PercentOutput, 0d, 0d);
    }

    protected void setMotors(ControlMode mode, double lVal, double rVal) {
        leftMotor.set(mode, lVal);
        rightMotor.set(mode, rVal);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
    }

    public DifferentialDrive getDrive() { return drive; }

    public TurnController getTurnController() { return turnController; }

}
