// ===================================================================
// FILE NAME: BallCounter.java (Team 339 - Kilroy)
//
// CREATED ON: Feb 12, 2022
// CREATED BY: Jacob Fisher
// MODIFIED ON: Feb 22, 2022
// MODIFIED BY: Jacob Fisher
// ABSTRACT:
// This file contains all of the global definitions for the
// ball counter related items in the system
// (probably just the one BallCounter integer)
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package frc.Utils;

import frc.HardwareInterfaces.MomentarySwitch;

public class BallCounter
    {
        
    // BALL COUNTS
    public int BallCount = 0;
    private int MinimumBallCount = 0;
    private int MaximumBallCount = 2;

    // MOMENTARY SWITCHES
    private MomentarySwitch addBallButton = null;
    private MomentarySwitch subtractBallButton = null;

    // -----------------------------------------------------

    /**
     * constructor
     *
     * NO PARAMS - UNRECOMMENDED 
     * 
     * @author Jacob Fisher
     * @written Feb 22, 2022
     */
    public BallCounter()
        {

        }
    
    /**
     * constructor
     *
     * @param setMinimumBallCount the minimum the ball count can be
     * @param setMaximumBallCount the maximum the ball count can be
     * 
     * @author Jacob Fisher
     * @written Feb 22, 2022
     */
    public BallCounter(final int setMinimumBallCount, final int setMaximumBallCount)
        {
            if (!(setMinimumBallCount > setMaximumBallCount)) MinimumBallCount = setMinimumBallCount;
            if (!(setMaximumBallCount < setMinimumBallCount)) MaximumBallCount = setMaximumBallCount;
        }
    
    /**
     * constructor
     *
     * @param setAddBallButton the add ball button from Hardware
     * @param setSubtractBallButton the subtract ball button from Hardware
     * 
     * @author Jacob Fisher
     * @written Feb 22, 2022
     */
    public BallCounter(final MomentarySwitch setAddBallButton, final MomentarySwitch setSubtractBallButton)
        {
            addBallButton = setAddBallButton;
            subtractBallButton = setSubtractBallButton;
        }

    /**
     * constructor
     *
     * @param setMinimumBallCount the minimum the ball count can be
     * @param setMaximumBallCount the maximum the ball count can be
     * @param setAddBallButton the add ball button from Hardware
     * @param setSubtractBallButton the subtract ball button from Hardware
     * 
     * @author Jacob Fisher
     * @written Feb 22, 2022
     */
    public BallCounter(final int setMinimumBallCount, final int setMaximumBallCount, final MomentarySwitch setAddBallButton, final MomentarySwitch setSubtractBallButton)
        {
            if (!(setMinimumBallCount > setMaximumBallCount)) MinimumBallCount = setMinimumBallCount;
            if (!(setMaximumBallCount < setMinimumBallCount)) MaximumBallCount = setMaximumBallCount;
            addBallButton = setAddBallButton;
            subtractBallButton = setSubtractBallButton;
        }

    /**
     * Get the {@link BallCounter#MinimumBallCount} currently set
     * 
     * @return int MinimumBallCount
     */
    public int getMinimumBallCount() 
    {
        return MinimumBallCount;
    }

    /**
     * Get the {@link BallCounter#MaximumBallCount} currently set
     * 
     * @return int MaximumBallCount
     */
    public int getMaximumBallCount() 
    {
        return MaximumBallCount;
    }

    /**
     * Set the {@link BallCounter#MinimumBallCount}
     * 
     * @param newMinimum The new minimum ball count
     * 
     * @return int MinimumBallCount
     */
    public int setMinimumBallCount(final int newMinimum) 
    {
        MinimumBallCount = newMinimum;
        return MinimumBallCount;
    }

    /**
     * Set the {@link BallCounter#MaximumBallCount}
     * 
     * @param newMaximum The new maximum ball count
     * 
     * @return int MaximumBallCount
     */
    public int setMaximumBallCount(final int newMaximum) 
    {
        MaximumBallCount = newMaximum;
        return MaximumBallCount;
    }

    /**
     * Set the new {@link BallCounter#addBallButton}
     * 
     * @param newAddBallButton The MomentarySwitch for the additive ball button
     * 
     * @return MomentarySwitch addBallButton
     */
    public MomentarySwitch setAddBallButton(final MomentarySwitch newAddBallButton)
    {
        addBallButton = newAddBallButton;
        return addBallButton;
    }

    /**
     * Set the new {@link BallCounter#subtractBallButton}
     * 
     * @param newSubtractBallButton The MomentarySwitch for the subtractive ball button
     * 
     * @return MomentarySwitch subtractBallButton
     */
    public MomentarySwitch setSubtractBallButton(final MomentarySwitch newSubtractBallButton)
    {
        subtractBallButton = newSubtractBallButton;
        return subtractBallButton;
    }

    /**
     * Add to the {@link BallCounter#BallCount} by the integer {@code addBy}
     * <p></p>
     * <strong>This method does NOT check for the current Ball count between the minimum
     * and maximum, nor if the joystick buttons are active or not, see {@link BallCounter#addCheckCount}</strong>
     * 
     * @param addBy The amount to add by
     * 
     * @return int BallCount
     */
    public int uncheckedAdd(final int addBy)
    {
        BallCount = BallCount + addBy;
        return BallCount;
    }

    /**
     * Subtract from the {@link BallCounter#BallCount} by the integer {@code subtractBy}
     * <p></p>
     * <strong>This method does NOT check for the current Ball count between the minimum
     * and maximum, nor if the joystick buttons are active or not, see {@link BallCounter#subtractCheckCount}</strong>
     * 
     * @param addBy The amount to subtract by
     * 
     * @return int BallCount
     */
    public int uncheckedSubtract(final int subtractBy)
    {
        BallCount = BallCount - subtractBy;
        return BallCount;
    }

    /**
     * Add to the {@link BallCounter#BallCount} by the integer {@code addBy}
     * and check for the current min/max ball counts compared to the amount
     * you are trying to add by, and check the state of the joystick buttons
     * <p></p>
     * <strong>Recommended compared to {@link BallCounter#uncheckedAdd}</strong>
     * 
     * @param addBy The amount to add by
     * 
     * @return int BallCount
     */
    public int addCheckCount(final int addBy)
    {
        if (BallCount + addBy <= MaximumBallCount)
            {
            BallCount = BallCount + addBy;
            // Associated Switch goes off
            if (addBallButton != null) addBallButton.setValue(false);
            }
        if (subtractBallButton.isOn() == true)
            // Alt Switch goes off
            if (subtractBallButton != null) subtractBallButton.setValue(false);
        return BallCount;
    }

    /**
     * Subtract from the {@link BallCounter#BallCount} by the integer {@code subtractBy}
     * and check for the current min/max ball counts compared to the amount
     * you are trying to subtract by, and check the state of the joystick buttons
     * <p></p>
     * <strong>Recommended compared to {@link BallCounter#uncheckedSubtract}</strong>
     * 
     * @param subtractBy The amount to subtract by
     * 
     * @return int BallCount
     */
    public int subtractCheckCount(final int subtractBy)
    {
        if (BallCount - subtractBy >= MinimumBallCount)
            {
            BallCount = BallCount - subtractBy;
            // Associated Switch goes off
            if (subtractBallButton != null) subtractBallButton.setValue(false);
            }
        if (addBallButton.isOn() == true)
            // Alt Switch goes off
            if (addBallButton != null) addBallButton.setValue(false);
        return BallCount;
    }
    }
