/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

public class Flywheels extends PIDSubsystem {


    private SpeedController flywheels;
    private Potentiometer flywheelsPot;

    private final double kp = SmartDashboard.getNumber("kp-flywheels", 1);
    private final double ki = SmartDashboard.getNumber("ki-flywheels", 1);
    private final double kd = SmartDashboard.getNumber("kd-flywheels", 1);

    public Flywheels() {
        super("Flywheels", SmartDashboard.getNumber("kp-flywheels", 1), SmartDashboard.getNumber("ki-flywheels", 1), SmartDashboard.getNumber("kd-flywheels", 1));
        setAbsoluteTolerance(.2);
        getPIDController().setContinuous(false);
        getPIDController().enable();
        flywheels = new Victor(RobotMap.flywheels_flywheels.value);
        flywheelsPot = new AnalogPotentiometer(RobotMap.flywheels_potentiometer.value, 360d, 30d);
    }


    @Override
    public void initDefaultCommand() {

    }

    @Override
    protected double returnPIDInput() {
        return flywheelsPot.get();
    }

    @Override
    protected void usePIDOutput(double output) {
        flywheels.pidWrite(output);
    }

    public void stop() {
        disable();
        getPIDController().reset();
        flywheels.set(0);
    }

    public void enable() {
        getPIDController().enable();
    }

    public void disable() {
        getPIDController().disable();
    }

}
