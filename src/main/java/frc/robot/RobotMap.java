    /*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public enum RobotMap {

    drivetrain_leftMotor(0), 
    drivetrain_rightMotor(1),
    drivetrain_leftMotorFollower(2), 
    drivetrain_rightMotorFollower(3),

    oi_leftStick(0),
    oi_rightStick(0);

    public final int value;

    RobotMap(int value) {
        this.value = value;
    }
    
}
