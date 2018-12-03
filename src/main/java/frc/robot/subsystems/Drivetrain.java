/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Util;
import frc.robot.commands.TankDrive;

public class Drivetrain extends Subsystem {

    private TalonSRX leftMotor, rightMotor, leftMotorFollower, rightMotorFollower;

    public Drivetrain() {
        leftMotor = new TalonSRX(RobotMap.drivetrain_leftMotor.value);
        rightMotor = new TalonSRX(RobotMap.drivetrain_rightMotor.value);
        leftMotorFollower = new TalonSRX(RobotMap.drivetrain_leftMotorFollower.value);
        rightMotorFollower = new TalonSRX(RobotMap.drivetrain_rightMotorFollower.value);

        Util.initTalon(leftMotor, rightMotor, leftMotorFollower, rightMotorFollower);

        leftMotorFollower.follow(leftMotor);
        rightMotorFollower.follow(rightMotor);
    }

    public void setMotors(ControlMode mode, double lVal, double rVal) {
        leftMotor.set(mode, lVal);
        rightMotor.set(mode, rVal);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
    }

}
