/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.Util;
import frc.robot.commands.TankDrive;

public class Drivetrain extends Subsystem implements PIDOutput {

    private TalonSRX leftMotor, rightMotor, leftMotorFollower, rightMotorFollower;
    private final AHRS ahrs;

    public final PIDController turnController;

    private final double kp = 0;
    private final double ki = 0;
    private final double kd = 0;

    public Drivetrain() {
        leftMotor = new TalonSRX(RobotMap.drivetrain_leftMotor.value);
        rightMotor = new TalonSRX(RobotMap.drivetrain_rightMotor.value);
        leftMotorFollower = new TalonSRX(RobotMap.drivetrain_leftMotorFollower.value);
        rightMotorFollower = new TalonSRX(RobotMap.drivetrain_rightMotorFollower.value);
        ahrs = new AHRS(Port.kMXP);

        Util.initTalon(leftMotor, rightMotor, leftMotorFollower, rightMotorFollower);

        leftMotorFollower.follow(leftMotor);
        rightMotorFollower.follow(rightMotor);

        turnController = new PIDController(kp, ki, kd, ahrs, this);
        turnController.setInputRange(-180d, 180d);
        turnController.setOutputRange(-.5d, .5d);
        turnController.setAbsoluteTolerance(3);
        turnController.setContinuous();
    }

    public void rotateDegrees(double angle) {
        ahrs.reset();
        turnController.reset();
        turnController.setPID(kp, ki, kd);
        turnController.setSetpoint(angle);
        turnController.enable();
    }

    public void setMotors(ControlMode mode, double lVal, double rVal) {
        leftMotor.set(mode, lVal);
        rightMotor.set(mode, rVal);
    }

    @Override
    public void initDefaultCommand() {
        setDefaultCommand(new TankDrive());
    }

    @Override
    public void pidWrite(double output) {
        setMotors(ControlMode.PercentOutput, -output, output);
    }

}
