/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.commands.RunFlywheels;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    
    public static final double JOYSTICK_THRESHOLD = .05;

    public final Joystick leftStick = new Joystick(RobotMap.oi_leftStick.value);
    public final Joystick rightStick = new Joystick(RobotMap.oi_rightStick.value);

    public OI() {
        Button rightTrigger = new JoystickButton(rightStick, 3);
        rightTrigger.toggleWhenActive(new RunFlywheels(.5));
    }

    public double getLeftX() {
        double raw = leftStick.getX();
        return Math.abs(raw) < JOYSTICK_THRESHOLD ? 0d : raw;
    }

    public double getRightX() {
        double raw = rightStick.getX();
        return Math.abs(raw) < JOYSTICK_THRESHOLD ? 0d : raw;
    }

    public double getLeftY() {
        double raw = leftStick.getY();
        return Math.abs(raw) < JOYSTICK_THRESHOLD ? 0d : raw;
    }

    public double getRightY() {
        double raw = rightStick.getY();
        return Math.abs(raw) < JOYSTICK_THRESHOLD ? 0d : raw;
    }

}
