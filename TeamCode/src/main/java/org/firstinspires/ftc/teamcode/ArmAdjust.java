package org.firstinspires.ftc.teamcode;

/**
 * Created by Miranda on 10/20/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * T
 */

@TeleOp(name="Adjust Arm", group="TeleOp")
//@Disabled
public class ArmAdjust extends LinearOpMode {

    /* Declare OpMode members. */
    THardware1 robot           = new THardware1();   // Use a hardware
    ElapsedTime runtime = new ElapsedTime();

    //Double for more precision
    @Override
    public void runOpMode() throws InterruptedException {


        double max;

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        ArtArm reach = new ArtArm(14.5, 15.75, 3, 1, 1, 2);
        robot.init(hardwareMap);

        int base = 0;
        int joint = 0;

        double[] pos = {0, 0, 0};

        double x = 2;
        double y = 1;
        double wrist = robot.EndJoint.getPosition();
        double grip = robot.Gripper.getPosition();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            ////////////////////////////////////////////ARTICULATED ARM/////////////////////////////////////////////////////////
            /*
            base += (int) gamepad1.right_stick_y * 10;
            joint += (int) gamepad1.right_stick_x * 10;
            wrist -= gamepad1.left_stick_y * 0.01;

            if (wrist >= 1)
                wrist = 1;
            else if (wrist <= 0)
                wrist = 0;

            robot.ArmBase.setTargetPosition(base);
            robot.ArmBase.setPower(0.4);
            robot.ArmJoint.setTargetPosition(joint);
            robot.ArmJoint.setPower(0.4);

            robot.EndJoint.setPosition(wrist);
            */

            wrist -= gamepad1.left_stick_y * 0.01;

            if (wrist > 1)
                wrist = 1;
            else if (wrist < 0)
                wrist = 0;

            robot.EndJoint.setPosition(wrist);


            grip += gamepad1.left_stick_x * 0.01;

            if (grip > 1)
                grip = 1;
            else if (grip < 0)
                grip = 0;

            robot.Gripper.setPosition(grip);

            x -= gamepad1.right_stick_x * 0.5;


            if(x > reach.maxX() - 0.5)
                x = reach.maxX() - 0.5;
            if(x < 1)
                x = 1;


            y -= gamepad1.right_stick_y * 0.5;


            if(y > reach.maxY() - 0.5)
                y = reach.maxY() - 0.5;
            if(y < 1)
                y = 1;


            if(gamepad1.a){
                robot.Gripper.setPosition(0.55);
            } else if(gamepad1.b){
                robot.Gripper.setPosition(0.4);
            }

            pos = reach.goToXY(x, y);
            if(pos[0] > 0)
                pos[0] = 0;
            robot.ArmBase.setTargetPosition((int) pos[0]);
            robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmBase.setPower(1);
            robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
            robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.ArmJoint.setPower(1);

            telemetry.addData("X: ", reach.getX());
            telemetry.addData("Y: ", reach.getY());
            telemetry.addData(" " + pos[0] + " " + pos[1] + " " + pos[2], "");
            telemetry.addData("Base: ", robot.ArmBase.getCurrentPosition());
            telemetry.addData("Joint: ", robot.ArmJoint.getCurrentPosition());
            telemetry.addData("Servo: ", robot.EndJoint.getPosition() + "" + wrist);
            telemetry.update();
            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.

            robot.waitForTick(40);
        }
    }

    public void armMove(int base, int joint){
        robot.ArmBase.setTargetPosition(base+20);
        robot.ArmBase.setPower(0.1);
        robot.ArmJoint.setTargetPosition(joint-70);
        robot.ArmJoint.setPower(0.2);
        //robot.EndJoint.setPosition(reach.getEEDeg());
    }
}
