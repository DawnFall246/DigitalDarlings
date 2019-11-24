package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This is an interface
 */
public interface ArmHardware
{
    /* Public OpMode members. */

    DcMotor ArmBase  = null;
    DcMotor ArmJoint = null;

    Servo EndJoint = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;




}
