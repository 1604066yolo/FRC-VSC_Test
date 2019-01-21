/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class TurnController implements PIDOutput {

    private static double kp, ki, kd;

    private final PIDController controller;
    
    private static final AHRS ahrs =  new AHRS(Port.kMXP);

    private static final double ABSOLUTE_TOLERANCE = 3d;

    public TurnController() {
        kp = SmartDashboard.getNumber("TurnController kp", 1);
        ki = SmartDashboard.getNumber("TurnController ki", 1);
        kd = SmartDashboard.getNumber("TurnController kd", 1);
        controller = new PIDController(kp, ki, kd, ahrs, this);
        controller.setInputRange(-180d, 180d);
        controller.setOutputRange(-.5d, .5d);
        controller.setAbsoluteTolerance(ABSOLUTE_TOLERANCE);
        controller.setContinuous();
    }

    public void turnToAngle(double degrees) {
        ahrs.reset();
        controller.reset();
        controller.setSetpoint(degrees);
        controller.enable();
    }

    public boolean isAtEndPosition() {
        return controller.onTarget();
    }

    @Override
    public void pidWrite(double output) {
        Robot.m_drivetrain.setMotors(ControlMode.PercentOutput, output, output);
    }

}
