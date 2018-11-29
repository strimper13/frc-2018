/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team166.robot;

import frc.team166.chopshoplib.controls.ButtonJoystick;
import frc.team166.chopshoplib.controls.ButtonXboxController;

public class OI {
    // Creates joysticks
    public ButtonJoystick leftDriveStick;
    public ButtonJoystick rightDriveStick;
    public ButtonXboxController xBoxTempest;
    public ButtonXboxController xBoxDrive;

    public OI() {
        // defines the joysticks as joysticks and assigns left and right
        leftDriveStick = new ButtonJoystick(RobotMap.Controller.leftcontrol);
        rightDriveStick = new ButtonJoystick(RobotMap.Controller.rightcontrol);
        xBoxTempest = new ButtonXboxController(RobotMap.Controller.Xboxcontrol);
        xBoxDrive = new ButtonXboxController(RobotMap.Controller.xboxDrive);

    }
}
