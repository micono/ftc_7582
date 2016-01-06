/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.ArrayList;

/**
 * TeleOp Mode
 * <p>
 * Enables control of the robot via the gamepad
 */
public class AutonomousOp extends OpMode {

    // position of the claw servo
    double clawPosition;

    // amount to change the claw servo position by
    double clawDelta = 0.01;

    // position of the wrist servo
    double wristPosition;

    // amount to change the wrist servo position by
    double wristDelta = 0.01;

    // command for the continuous servo
    double continuousPosition;
    static double continuousStop = 0.5;
    double continuousDelta = 0.05;

    DcMotorController.DeviceMode devMode;
    DcMotorController wheelControllerFront;
    DcMotorController wheelControllerRear;
    DcMotor motorRightFront;
    DcMotor motorLeftFront;
    DcMotor motorRightRear;
    DcMotor motorLeftRear;
    DcMotor rodMotor;
    DcMotor dadMotor;

    Servo dadTilt;
    Servo claw;
    Servo wrist;
    Servo continuous;

    int[] start = {0, 100};

    int numOpLoops = 1;

    public AutonomousOp()
    {
        DbgLog.msg("FourWheelDriveOp");
    }
    /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init()
     */
    @Override
    public void init() {
        motorRightFront = hardwareMap.dcMotor.get("motor_rt_front");
        motorRightRear = hardwareMap.dcMotor.get("motor_rt_rear");
        motorLeftFront = hardwareMap.dcMotor.get("motor_lt_front");
        motorLeftRear = hardwareMap.dcMotor.get("motor_lt_rear");

        claw = hardwareMap.servo.get("claw"); // channel 6
        wrist = hardwareMap.servo.get("wrist"); // channel 1
        dadTilt = hardwareMap.servo.get("dad_tilt");
        continuous = hardwareMap.servo.get("continuous");

        wheelControllerFront = hardwareMap.dcMotorController.get("wheels_front");
        wheelControllerRear = hardwareMap.dcMotorController.get("wheels_rear");

        rodMotor = hardwareMap.dcMotor.get("rod_motor");
        dadMotor = hardwareMap.dcMotor.get("dad_motor");
    }

    /*
     * Code that runs repeatedly when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#init_loop()
     */
    @Override
    public void init_loop() {

        devMode = DcMotorController.DeviceMode.WRITE_ONLY;

        /// only the motor on the left side runs in reverse
        ///
        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorLeftRear.setDirection(DcMotor.Direction.REVERSE);

        // set the mode
        // Nxt devices start up in "write" mode by default, so no need to switch device modes here.
        motorLeftFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorLeftRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightFront.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        motorRightRear.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);

        wristPosition = 0.6;
        clawPosition = 0.5;
    }

    /*
     * This method will be called repeatedly in a loop
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
     */
    @Override
    public void loop() {


        // The op mode should only use "write" methods (setPower, setMode, etc) while in
        // WRITE_ONLY mode or SWITCHING_TO_WRITE_MODE
        if (allowedToWrite()) {
    /*
     * Gamepad 1
     *
     * Gamepad 1 controls the motors via the left stick, and it controls the wrist/claw via the a,b,
     * x, y buttons
     * /

    /*
     * Gamepad 2
     *
     * Gamepad controls the motors via the right trigger as a throttle, left trigger as reverse, and
     * the left stick for direction. This type of control is sometimes referred to as race car mode.
     */

            // To read any values from the NXT controllers, we need to switch into READ_ONLY mode.
            // It takes time for the hardware to switch, so you can't switch modes within one loop of the
            // op mode. Every 17th loop, this op mode switches to READ_ONLY mode, and gets the current power.
            if (numOpLoops % 17 == 0) {
                // Note: If you are using the NxtDcMotorController, you need to switch into "read" mode
                // before doing a read, and into "write" mode before doing a write. This is because
                // the NxtDcMotorController is on the I2C interface, and can only do one at a time. If you are
                // using the USBDcMotorController, there is no need to switch, because USB can handle reads
                // and writes without changing modes. The NxtDcMotorControllers start up in "write" mode.
                // This method does nothing on USB devices, but is needed on Nxt devices.

                syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode.READ_ONLY);
            }

            // Every 17 loops, switch to read mode so we can read data from the NXT device.
            // Only necessary on NXT devices.
            if (wheelControllerFront.getMotorControllerDeviceMode() == DcMotorController.DeviceMode.READ_ONLY) {

                // Update the reads after some loops, when the command has successfully propagated through.
                telemetry.addData("Text", "free flow text");

                telemetry.addData("RunMode: ", motorLeftFront.getMode().toString());

                /// looks to me like setting the DeviceMode to WRITE_ONLY takes some time
                /// this won't return until the transition is complete
                ///
                int delayms = syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

                telemetry.addData("Status OPL/Delay ", String.valueOf(numOpLoops) + " / " + String.valueOf(delayms));
                // Reset the loop
                // numOpLoops = 0;
                // Only needed on Nxt devices, but not on USB devices
                //wheelController.setMotorControllerDeviceMode(DcMotorController.DeviceMode.WRITE_ONLY);

                // Reset the loop
                //numOpLoops = 0;
            }

            // Update the current devMode
            devMode = wheelControllerFront.getMotorControllerDeviceMode();
            numOpLoops++;

        }
    }

    // If the device is in either of these two modes, the op mode is allowed to write to the HW.
    private boolean allowedToWrite(){
        return (devMode == DcMotorController.DeviceMode.WRITE_ONLY);
    }

    /// the DcMotorControllers take a while to switch modes. we can't afford to have
    /// have the controller to switch from WRITE_ONLY to READ_ONLY in the middle of
    /// move or (less importantly) switch from READ_ONLY to WRITE_ONLY in the middle
    /// of gathering telemetry. Use this method to change the mode and not return until
    /// the transition is done. It checks every 10ms for a transition
    ///
    private int syncSetMotorControllerDeviceMode(DcMotorController.DeviceMode newMode)
    {
        DcMotorController.DeviceMode devMode;
        int delay = 0;

        while (true)
        {
            wheelControllerFront.setMotorControllerDeviceMode(newMode);
            wheelControllerRear.setMotorControllerDeviceMode(newMode);

            /// sleep for 10ms and check
            ///
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                /// sleep got interrupted - oh well
                e.printStackTrace();
            }
            delay++;

            devMode = wheelControllerFront.getMotorControllerDeviceMode();

            if (devMode == newMode)
            { break; }
        }

        return delay * 10;
    }

    private boolean isScheduledTime(){
        boolean yes = false;
        for (int i = 0; i < start.length; i++){
            if (start[i] == numOpLoops){
                yes = true;
                break;
            }
        }
        return yes;
    }
}