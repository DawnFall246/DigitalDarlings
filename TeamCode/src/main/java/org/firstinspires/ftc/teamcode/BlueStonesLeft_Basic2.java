/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;


@Autonomous(name = "Blue Stones Left Basic 2", group = "Auto")
//@Disabled
public class BlueStonesLeft_Basic2 extends LinearOpMode {

    AHardware3 robot = new AHardware3();
    ArtArm arm = new ArtArm(14.5, 15.75, 3, 1, 1, 2);
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime waitTime = new ElapsedTime();
    private ElapsedTime totalTime = new ElapsedTime();

    Orientation angles;
    Acceleration gravity;

    // IMPORTANT: If you are using a USB WebCam, you must select CAMERA_CHOICE = BACK; and PHONE_IS_PORTRAIT = false;
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AaarmqH/////AAABmbHOlxYNNkj2t8qVDAEWlbxenas9EeW8QU" +
            "crqOfItManosKlM+R7VcYB8pmOjbgOej+tVoxYZg8m8EnoAW/Ip3ZpShhokDXsGZY0QLEW/VeUm0puKTMDxv" +
            "u22/NdA+LbOGsC3i26kdI8iL0kDr7KKzksE7dcpF1HGpjCD1mTTevrxQVQKo0F0AgPIHCnszH+oirl934NVr" +
            "0bn6TsU2eg5IyfB66CzfyTxA45r5yq7pCYd0XGPOfbhwGvuc14x4gtV9Z89MXmHHejH+3BsyaNb1K2/o/tDb" +
            "xEo/dn8VIa/PVedI3n/1ei/sHlmrm5ja2EgTOZ3E9+18ZIZ6V2+2WARaQJQxe1NrwPctkpwo44ybZx\n";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    VectorF stonePos;
    int config;

    //Parameters
    double[] pos = new double[3];
    double servoPos = 0;
    final double servoRange = (1.35*Math.PI);
    double basePower = 1.0;
    double jointPower = 1.0;

    static double yaw_intended;

    final static double[] underPos = {2, 1.5, 0.4};
    final static double[] grabPos = {3, 3.5, 0};
    final static double[] liftPos = {17, 12, 0};
    final static double[] lowerPos = {17, 8.5, 0};

    final static double CountsPerInchFB = 15000.0/59.0;
    final static double CountsPerInchLR = 10000.0/34.0;
    static double fullSpeedSlowFB = 8.5;
    static double fullSpeedSlowLR = 2.0;
    final static double displacementGain = 1;

    final static int[] RED = {132, 101, 25, 29};
    final static int[] BLUE = {152, 32, 54, 74};
    final static int[] MAT = {89, 33, 32, 28};




    @Override
    public void runOpMode() {

        /********************************************************** VUFORIA ***********************************************************/

        //Retrieve the camera we are to use.

        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        /* Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;


        //We also indicate which camera on the RC we wish to use
        parameters.cameraName = webcamName;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Load the data sets for the trackable objects. These particular data
        // sets are stored in the 'assets' part of our application.
        VuforiaTrackables targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");
        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");

        // For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90 ;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT  = -7.0f * mmPerInch;   // eg: Camera is 7 Inches behind gripper
        final float CAMERA_VERTICAL_DISPLACEMENT = 13.0f * mmPerInch;   // eg: Camera is 13 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT     = 8.0f * mmPerInch;     // eg: Camera is 7 inches left of the gripper

        OpenGLMatrix robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        /**  Let all the trackable listeners know where the phone is.  */
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(robotFromCamera, parameters.cameraDirection);
        }

        updateVuforia(allTrackables);

        //initVuforia();

        robot.init(hardwareMap);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        yaw_intended = angles.thirdAngle;
        /********************************************************** START ***********************************************************/

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        runtime.reset();
        waitForStart();
        totalTime.reset();
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        targetsSkyStone.activate();
        /*
        PIDMove(0, 1, 20, 0.01, 0, 0.01, false);
        sleep(20000);
        PIDMove(0, 1, 40, 0.01, 0, 0.01, false);
        turnTo(90, 1, 0.5);
        sleep(2000);
        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        telemetry.addData("Theta", angles.thirdAngle);
        telemetry.update();
        sleep(5000);

        PIDMove(90, 1, 10000, 0, 0, 0.01, false);
        sleep(5000);
        */


        //Move forward 18”
        PIDMove(0, 1, 20, 0.01, 0, 0.01,false);
        //Strafe 24” left
        PIDMove(90, 1, 20, 0.01, 0, 0.01,  false);
        //Pause
        stonePos = updateVuforia(allTrackables);
        //If notDetected
        if(stonePos.get(2) == -1){
            //Strafe 5” left
            PIDMove(90, 0.3, 7.5, 0.01, 0, 0.01, false);
        } else {
            //Strafe so arm is centered
            telemetry.addData("displacement", CAMERA_LEFT_DISPLACEMENT / mmPerInch);
            telemetry.addData("move", (stonePos.get(1) - CAMERA_LEFT_DISPLACEMENT) / mmPerInch);
            telemetry.update();
            PIDMove(270, 0.3, (stonePos.get(1) - CAMERA_LEFT_DISPLACEMENT) / mmPerInch * displacementGain,
                    0.01, 0, 0.01, false);
        }
        //Open gripper
        pos = arm.goToXY(grabPos[0], grabPos[1]);
        servoPos = grabPos[2];
        if(pos[0] > 0)
            pos[0] = 0;
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmBase.setPower(basePower);
        robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
        robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmJoint.setPower(jointPower);
        if(servoPos == 0)
            servoPos = (2.0*Math.PI - pos[2])/servoRange;
        if(servoPos >= 0 && servoPos <= 1)
            robot.EndJoint.setPosition(servoPos);
        else if(servoPos < 0)
            robot.EndJoint.setPosition(0);
        else if(servoPos > 1)
            robot.EndJoint.setPosition(1);
        else
            robot.EndJoint.setPosition(0.5);
        robot.Gripper.setPosition(0.0);
        sleep(500);
        //Move forward 10”
        PIDMove(0, 0.6, 10, 0.01, 0, 0.01, false);
        //Grab stone
        robot.Gripper.setPosition(1.0);
        sleep(500);
        //Under bridge position
        pos = arm.goToXY(underPos[0], underPos[1]);
        servoPos = underPos[2];
        if(pos[0] > 0)
            pos[0] = 0;
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmBase.setPower(basePower);
        robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
        robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmJoint.setPower(jointPower);
        if(servoPos == 0)
            servoPos = (2.0*Math.PI - pos[2])/servoRange;
        if(servoPos >= 0 && servoPos <= 1)
            robot.EndJoint.setPosition(servoPos);
        else if(servoPos < 0)
            robot.EndJoint.setPosition(0);
        else if(servoPos > 1)
            robot.EndJoint.setPosition(1);
        else
            robot.EndJoint.setPosition(0.5);
        //Move back 13”
        PIDMove(180, 1, 2, 0.01, 0, 0.01, false);
        //Turn to 90 deg
        turnTo(90, 0.9, 1);
        //Move forward 43” while lifting arm
        if(stonePos.get(2) == -1)
            PIDMove(0, 1, 80, 0.01, 0.01, 0.01, false);
        else
            PIDMove(0, 1, 80 + ((stonePos.get(1) - CAMERA_LEFT_DISPLACEMENT) / mmPerInch * displacementGain),
                    0.01, 0.01, 0.01, false);
        turnTo(0, 0.9, 1);

        pos = arm.goToXY(liftPos[0], liftPos[1]);
        servoPos = liftPos[2];
        if(pos[0] > 0)
            pos[0] = 0;
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmBase.setPower(basePower);
        robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
        robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmJoint.setPower(jointPower);
        if(servoPos == 0)
            servoPos = (2.0*Math.PI - pos[2])/servoRange;
        if(servoPos >= 0 && servoPos <= 1)
            robot.EndJoint.setPosition(servoPos);
        else if(servoPos < 0)
            robot.EndJoint.setPosition(0);
        else if(servoPos > 1)
            robot.EndJoint.setPosition(1);
        else
            robot.EndJoint.setPosition(0.5);
        sleep(500);
        //Drop stone
        pos = arm.goToXY(lowerPos[0], lowerPos[1]);
        servoPos = lowerPos[2];
        if(pos[0] > 0)
            pos[0] = 0;
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmBase.setPower(basePower);
        robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
        robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmJoint.setPower(jointPower);
        if(servoPos == 0)
            servoPos = (2.0*Math.PI - pos[2])/servoRange;
        if(servoPos >= 0 && servoPos <= 1)
            robot.EndJoint.setPosition(servoPos);
        else if(servoPos < 0)
            robot.EndJoint.setPosition(0);
        else if(servoPos > 1)
            robot.EndJoint.setPosition(1);
        else
            robot.EndJoint.setPosition(0.5);
        //waitUntilPosition(robot.ArmJoint, 2, 15);
        sleep(500);
        robot.Gripper.setPosition(0.0);
        sleep(500);
        //Under bridge position
        pos = arm.goToXY(underPos[0], underPos[1]);
        servoPos = underPos[2];
        if(pos[0] > 0)
            pos[0] = 0;
        robot.ArmBase.setTargetPosition((int) pos[0]);
        robot.ArmBase.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmBase.setPower(basePower);
        robot.ArmJoint.setTargetPosition(-1 * (int) pos[1]);
        robot.ArmJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.ArmJoint.setPower(jointPower);
        if(servoPos == 0)
            servoPos = (2.0*Math.PI - pos[2])/servoRange;
        if(servoPos >= 0 && servoPos <= 1)
            robot.EndJoint.setPosition(servoPos);
        else if(servoPos < 0)
            robot.EndJoint.setPosition(0);
        else if(servoPos > 1)
            robot.EndJoint.setPosition(1);
        else
            robot.EndJoint.setPosition(0.5);

        //Turn to 270
        turnTo(270, .9,  1);
        //Forward to line
        PIDMove(0, 0.7, 0, 0.01, 0, 0.01, true);

        telemetry.update();
        telemetry.addData("Time", totalTime.seconds());
        telemetry.update();
        sleep(5000);





    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true);
        vuforia.setFrameQueueCapacity(1);

    }

    private VectorF updateVuforia(List<VuforiaTrackable> allTrackables){
        waitTime.reset();
        while (waitTime.seconds() < 2) {

            // check all the trackable targets to see which one (if any) is visible.
            targetVisible = false;
            for (VuforiaTrackable trackable : allTrackables) {
                if (((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible()) {
                    telemetry.addData("Visible Target", trackable.getName());
                    targetVisible = true;
                    // getUpdatedRobotLocation() will return null if no new information is available since
                    // the last time that call was made, or if the trackable is not currently visible.
                    OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();
                    if (robotLocationTransform != null) {
                        lastLocation = robotLocationTransform;
                    }
                    break;
                }
            }

            // Provide feedback as to where the robot is located (if we know).
            if (targetVisible) {
                // express position (translation) of robot in inches.
                VectorF translation = lastLocation.getTranslation();
                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f",
                        rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
                return translation;
            }
            else {
                telemetry.addData("Visible Target", "none");
            }
            telemetry.update();
        }
        return new VectorF(-1, 5, -1);
    }

    public void PIDMove(double dir, double speed, double dist, double Kp, double Ki, double dt, boolean toColor){
        /*
        dir = approx direction in degrees to move relative to forward face of robot.
        speed = speed to move [0,1]
        dist = approx distance to move in inches.
        Kp = proportional gain > 0
        Ki = integral gain >= 0
        dt = is a time step to control how fast the loop runs.
        */
        dir = dir*Math.PI / 180.0;
        //dist = dist * CountsPerInch;
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(500);

        int FRinit = robot.MFR.getCurrentPosition();
        int FLinit = robot.MFL.getCurrentPosition();
        int BRinit = robot.MBR.getCurrentPosition();
        int BLinit = robot.MBL.getCurrentPosition();
        double drive = speed*Math.cos(dir);
        double strafe = speed*Math.sin(dir);

        //double yaw_init = angles.thirdAngle; //get current yaw angle when the move is requested.  Goal is to keep this angle constant.
        double estX, estY, estDist;
        double yaw, yawErr, yawErrI;
        double turn = 0;
        int count = 0;

        int red, green, blue, alpha;
        double dR, dB, dM;

        boolean notFinished = false;
        double used_speed = speed;

        double Dslow = speed * (fullSpeedSlowFB * Math.cos(dir) + fullSpeedSlowLR * Math.sin(dir));

        do{
            count ++;
            //get motor encoder values and compute estimated distance, estDist
            estX = ( (robot.MFR.getCurrentPosition()-FRinit) + (robot.MFL.getCurrentPosition()-FLinit) +
                    (robot.MBR.getCurrentPosition()-BRinit) + (robot.MBL.getCurrentPosition()-BLinit)) * Math.cos(45*Math.PI/180.0) / CountsPerInchFB;
            estY = (-(robot.MFR.getCurrentPosition()-FRinit) + (robot.MFL.getCurrentPosition()-FLinit) +
                    (robot.MBR.getCurrentPosition()-BRinit) - (robot.MBL.getCurrentPosition()-BLinit)) * Math.sin(45*Math.PI/180.0) / CountsPerInchLR;
            estDist = Math.sqrt(Math.pow(estX, 2) + Math.pow(estY, 2));

            yawErrI= 0;
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw = angles.thirdAngle;
            yawErr = yaw - yaw_intended;  //note, some math may be needed so that yawErr does not make 360 deg jumps.
            // Alternatively, set gyro yaw to zero instead of measuring ywa_init

            if(Math.abs(yawErr) < 45) { //prevent 180 wraparound, but also limit how how big this term can get.
                yawErrI = yawErrI + yawErr * dt;  //this is effectively the integral of yaw error
                turn = Kp * (yawErr) + Ki * (yawErrI);  //correction you make to compensate for the twist of the foundation
            }
            if (Math.abs(turn) > 1.0) {
                turn = turn / Math.abs(turn);
            }

            if(!toColor) {
                if ( (Math.abs(dist) - Math.abs(estDist)) < Dslow )
                    used_speed = 0.2;
            }

            moveWithEnc(drive, strafe, turn, used_speed);
            telemetry.addData("Drive", drive);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("Turn", turn);
            telemetry.addData("Dslow", Dslow);
            telemetry.addData("Speed", used_speed);
            telemetry.addData("Yaw", yaw);
            telemetry.addData(dist + " ", estDist);
            telemetry.addData("", count);
            telemetry.update();

            if(toColor) {
                //compute 'distance' from current color sensor reading to RED, BLUE and MAT
                red = robot.Color.red();
                green = robot.Color.green();
                blue = robot.Color.blue();
                alpha = robot.Color.alpha();

                dR = Math.sqrt(Math.pow((alpha - RED[0]), 2) + Math.pow((red - RED[1]), 2)
                        + Math.pow((green - RED[2]), 2) + Math.pow((blue - RED[3]), 2));
                dB = Math.sqrt(Math.pow((alpha - BLUE[0]), 2) + Math.pow((red - BLUE[1]), 2)
                        + Math.pow((green - BLUE[2]), 2) + Math.pow((blue - BLUE[3]), 2));
                dM = Math.sqrt(Math.pow((alpha - MAT[0]), 2) + Math.pow((red - MAT[1]), 2)
                        + Math.pow((green - MAT[2]), 2) + Math.pow((blue - MAT[3]), 2));
                telemetry.addData("Alpha: ", alpha);
                telemetry.addData("Red: ", red);
                telemetry.addData("Green: ", green);
                telemetry.addData("Blue: ", blue);
                telemetry.addData("dB < dM = ", dB < dM*2.5);
                telemetry.addData("dR < dM = ", dR < dM*2.5);

                notFinished = opModeIsActive() && dB > dM*2.5 && dR > dM*2.5;// && runtime.seconds() < 6;
                telemetry.addData("Finished?", !notFinished);
                telemetry.update();
            }
            else
                notFinished = opModeIsActive() && Math.abs(estDist) < Math.abs(dist);

            sleep((int) (dt*1000));
        } while(notFinished);
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        sleep(1);
    }

    private void moveWithEnc(double drive, double strafe, double turn, double speed){
        double FR =  drive - turn -  strafe; //+-
        double BR =  drive - turn +  strafe; //++
        double FL =  drive + turn +  strafe; //-+
        double BL =  drive + turn -  strafe; //--

        double Mmax = Math.max(Math.max(Math.abs(FR), Math.abs(BR)), Math.max(Math.abs(FL), Math.abs(BL)));

        FR =  speed*FR / Mmax;
        BR =  speed*BR / Mmax;
        FL =  speed*FL / Mmax;
        BL =  speed*BL / Mmax;

        robot.MFL.setPower(FL);
        robot.MBL.setPower(BL);
        robot.MFR.setPower(FR);
        robot.MBR.setPower(BR);
    }

    public void turn(double init_target_angle, double speed) {
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double yaw_theta = angles.thirdAngle;

        double target_angle = init_target_angle + yaw_theta;
        if(target_angle > 180)
            target_angle -= 360;
        else if(target_angle < -180)
            target_angle += 360;

        double left_power, right_power;

        while (Math.abs(yaw_theta - target_angle) > 1  &&  opModeIsActive()) {
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw_theta = angles.thirdAngle;
            if (init_target_angle > 0) { //Turn left
                left_power = -speed;
                right_power = speed;
            } else { //Turn right
                left_power = speed;
                right_power = -speed;
            }

            robot.MFL.setPower(left_power);
            robot.MBL.setPower(left_power);
            robot.MFR.setPower(right_power);
            robot.MBR.setPower(right_power);

            telemetry.addData("Angle: ", angles.thirdAngle);
            telemetry.addData("Diff: ", Math.abs(yaw_theta - target_angle));
            telemetry.update();
        }

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        yaw_intended = angles.thirdAngle;

        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);

        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnTo(double init_target_angle, double speed, double tolerance) {
        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);
        robot.MFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double yaw_theta = angles.thirdAngle;
        double target_angle = init_target_angle;
        if(init_target_angle > 180)
            target_angle -= 360;
        else if(init_target_angle < -180)
            target_angle += 360;

        double left_power, right_power;
        double used_speed = speed;

        while (Math.abs(yaw_theta - target_angle) > tolerance  &&  opModeIsActive()) {
            angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            yaw_theta = angles.thirdAngle;


            if(Math.abs(yaw_theta - target_angle) < 20)
                used_speed = 0.1;

            if (target_angle - yaw_theta < 0) { //Turn right(-)
                left_power  = used_speed;
                right_power = -used_speed;
            } else { //Turn left(+)
                left_power  = -used_speed;
                right_power = used_speed;
            }

            robot.MFL.setPower(left_power);
            robot.MBL.setPower(left_power);
            robot.MFR.setPower(right_power);
            robot.MBR.setPower(right_power);

            telemetry.addData("Angle: ", angles.thirdAngle);
            telemetry.addData("Target", target_angle);
            telemetry.addData("Init", init_target_angle);
            telemetry.addData("Diff: ", yaw_theta - target_angle);
            telemetry.addData("Speed", used_speed);
            telemetry.update();
        }

        angles = robot.IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        yaw_intended = angles.thirdAngle;

        robot.MFL.setPower(0);
        robot.MBL.setPower(0);
        robot.MFR.setPower(0);
        robot.MBR.setPower(0);

        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    public void waitUntilPosition(DcMotor motor, int sec, int tolerance){
        runtime.reset();
        while  (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > tolerance && opModeIsActive()){
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.update();
            if(runtime.seconds() >= sec)
                break;
        }
    }
    public void waitUntilWheelPosition(int tolerance){
        while((Math.abs(robot.MBR.getCurrentPosition() - robot.MBR.getTargetPosition()) > tolerance ||
                Math.abs(robot.MFL.getCurrentPosition() - robot.MFL.getTargetPosition()) > tolerance ||
                Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) > tolerance ||
                Math.abs(robot.MFR.getCurrentPosition() - robot.MFR.getTargetPosition()) > tolerance) &&
                opModeIsActive())
        {
            if(Math.abs(robot.MBL.getCurrentPosition() - robot.MBL.getTargetPosition()) <= tolerance){
                robot.MBL.setPower(0);
            }
            if(Math.abs(robot.MFL.getCurrentPosition() - robot.MFL.getTargetPosition()) <= tolerance){
                robot.MFL.setPower(0);
            }
            if(Math.abs(robot.MBR.getCurrentPosition() - robot.MBR.getTargetPosition()) <= tolerance){
                robot.MBR.setPower(0);
            }
            if(Math.abs(robot.MFR.getCurrentPosition() - robot.MFR.getTargetPosition()) <= tolerance){
                robot.MFR.setPower(0);
            }
        }
    }

    public void resetWheelEncoders(){
        robot.MFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.MBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.MFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.MBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sleep(100);
    }

    public void slowStop(DcMotor motor, int tolerance1, int tolerance2, int tolerance3){
        double power = motor.getPower();
        while(Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) > tolerance1 && opModeIsActive()) {
            telemetry.addData("Encoder", motor.getCurrentPosition());
            telemetry.addData("Target", motor.getTargetPosition());
            telemetry.update();
            if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < tolerance2 && opModeIsActive()) {
                motor.setPower(power / 4);
            }else if (Math.abs(motor.getCurrentPosition() - motor.getTargetPosition()) < tolerance3 && opModeIsActive()) {
                motor.setPower(power / 2);
            }
        }
    }
}