#include <gui/screen1_screen/Screen1View.hpp>
#include "cmsis_os.h"

Screen1View::Screen1View()
    : waitTimerActive(false),
      waitCounter(0),
      lastToggleState(false)
{
}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}


void Screen1View::enableButtonWithLabel(touchgfx::ButtonWithLabel &button)
{
	buttonInit.setAlpha(150);
	buttonStandby.setAlpha(150);
	buttonCharging.setAlpha(150);
	buttonDischarging.setAlpha(150);

	button.setAlpha(255);
	button.invalidate();
}

bool Screen1View::isSystemError()
{
//    float currentSensorVal = (float)PumpCurrentSensor_slider.getValue() / 100.0f;
//    float voltageSensorVal = (float)VoltageSensor_slider.getValue() / 100.0f;
//
////    if ((SysStateBgError.getAlpha()== 0) && (imagePumpError.getAlpha() == 0)){
////        return false;
//
//    if(currentSensorVal<1 || currentSensorVal >1.3){
//
//    	return true;
//    }else if(!((voltageSensorVal >0.9 && voltageSensorVal <1.3)|| (voltageSensorVal >2.1 && voltageSensorVal <2.5))){
//
//    	return true;
//    }

    if(isSystemPumpCurrentError() == true || isSystemVoltageError() == true)
    	return true;

    return false;
}

bool Screen1View::isSystemCharging()
{
    float voltageSensorVal = (float)VoltageSensor_slider.getValue() / 100.0f;
	if(isSystemError() == false)
	{
		if (voltageSensorVal >2.1 && voltageSensorVal <2.5){
			return true;
		}
	}

	return false;
}

bool Screen1View::isSystemVoltageError()
{
    float voltageSensorVal = (float)VoltageSensor_slider.getValue() / 100.0f;

    if(!((voltageSensorVal >0.9 && voltageSensorVal <1.3)|| (voltageSensorVal >2.1 && voltageSensorVal <2.5))){
        	return true;
    }

	return false;
}

bool Screen1View::isSystemPumpCurrentError()
{
    float currentSensorVal = (float)PumpCurrentSensor_slider.getValue() / 100.0f;

    if(currentSensorVal<1 || currentSensorVal >1.3){

    	return true;
    }

	return false;
}


void Screen1View::showPumpRunningStatus(bool bYes)
{
	if(bYes == true){
		//Hide idle status of pump
		text_PumpIdle.setAlpha(0);
		//Show pump is running
		text_PumpRunning.setAlpha(255);
	}else{
		text_PumpIdle.setAlpha(255);
		text_PumpRunning.setAlpha(0);
	}

	text_PumpIdle.invalidate();
	text_PumpRunning.invalidate();
}

void Screen1View::sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value)
{
	//touchgfx_printf("Change value: %d\n", value);
	float voltageSensorVal = (float)VoltageSensor_slider.getValue() / 100.0f;
    float currentSensorVal = (float)PumpCurrentSensor_slider.getValue() / 100.0f;


    if (&src == &PumpCurrentSensor_slider)
    {
        //PumpCurrentSensor_ChangeValue
        //When PumpCurrentSensor_slider value changed execute C++ code
        //Execute C++ code
        Unicode::snprintfFloat(
            textArea_CurrentSensorBuffer,
            TEXTAREA_CURRENTSENSOR_SIZE,
            "%.1f",
            currentSensorVal
        );

        //Error
        if(currentSensorVal<1 || currentSensorVal >1.3){
        	imagePumpError.setAlpha(255);
        	if(bSystemOn == true)
        	{
        		enableButtonWithLabel(buttonStandby); //error here
        	}

        	showPumpRunningStatus(false);
        }
        else
        {
        	//Disable error background
        	imagePumpError.setAlpha(0);

        	//Update state only system on
        	if(bSystemOn == true){

				if(voltageSensorVal >0.9 && voltageSensorVal <1.3)
				{
					enableButtonWithLabel(buttonDischarging);
				}
				else if (voltageSensorVal >2.1 && voltageSensorVal <2.5)
				{
					enableButtonWithLabel(buttonCharging);
				}else
				{
					enableButtonWithLabel(buttonStandby);
				}
        	}
        }

        textArea_CurrentSensor.invalidate();
        imagePumpError.invalidate();
    }
    if (&src == &VoltageSensor_slider)
    {
        //VotageSensor_ValueChange
        //When VoltageSensor_slider value changed execute C++ code
        //Execute C++ code
        Unicode::snprintfFloat(
            textArea_VoltageSensorBuffer,
            TEXTAREA_VOLTAGESENSOR_SIZE,
            "%.1f",
            (float)VoltageSensor_slider.getValue() / 100.0f
        );
        textArea_VoltageSensor.invalidate();

        //VoltageSensorSlider_ChangeValue
        //When VoltageSensor_slider value changed execute C++ code
        //Execute C++ code
        float voltageSensorVal =     (float)VoltageSensor_slider.getValue() / 100.0f;

        Unicode::snprintfFloat(
            textArea_VoltageSensorBuffer,
            TEXTAREA_VOLTAGESENSOR_SIZE,
            "%.1f",
            voltageSensorVal
        );

        if(voltageSensorVal >0.9 && voltageSensorVal <1.3)
        {
        	SysStateBgError.setAlpha(0);

            if(isSystemError() == false && bSystemOn == true){
            	enableButtonWithLabel(buttonDischarging);
            	showPumpRunningStatus(true);
            }
        }
        else if (voltageSensorVal >2.1 && voltageSensorVal <2.5)
		{
        	SysStateBgError.setAlpha(0);

            if(isSystemError() == false && bSystemOn == true){
            	enableButtonWithLabel(buttonCharging);
            	showPumpRunningStatus(true);
            }
		}else
        {
        	SysStateBgError.setAlpha(255);
        	if(bSystemOn == true)
        		enableButtonWithLabel(buttonStandby);
        }

        textArea_VoltageSensor.invalidate();
        SysStateBgError.invalidate();
    }
}

void Screen1View::sliderValueConfirmedCallbackHandler(const touchgfx::Slider& src, int value)
{
	float voltageSensorVal = (float) value/ 100.0f ; //   	(float)VoltageSensor_slider.getValue() / 100.0f;
    float currentSensorVal = (float) value/ 100.0f ; //		(float)PumpCurrentSensor_slider.getValue() / 100.0f;

    //touchgfx_printf("Confirmed value: %d\n", value);

    if (&src == &PumpCurrentSensor_slider)
    {
        //PumpCurrentSensorSlider_AdjConfirmed
        //When PumpCurrentSensor_slider value confirmed execute C++ code
        //Execute C++ code
        Unicode::snprintfFloat(
            textArea_CurrentSensorBuffer,
            TEXTAREA_CURRENTSENSOR_SIZE,
            "%.1f",
			currentSensorVal
            //(float)PumpCurrentSensor_slider.getValue() / 100.0f
        );
        textArea_CurrentSensor.invalidate();
    }
    if (&src == &VoltageSensor_slider)
    {
        //VoltageSensorSlider_AdjConfirmed
        //When VoltageSensor_slider value confirmed execute C++ code
        //Execute C++ code
        Unicode::snprintfFloat(
            textArea_VoltageSensorBuffer,
            TEXTAREA_VOLTAGESENSOR_SIZE,
            "%.1f",
			voltageSensorVal
            //(float)VoltageSensor_slider.getValue() / 100.0f
        );
        textArea_VoltageSensor.invalidate();
    }

    if(bSystemOn == false)
    	return;

    //Update state
    if(isSystemError() == false)
    {
    	//Show pump is running
    	showPumpRunningStatus(true); //bug bug

        if(voltageSensorVal >0.9 && voltageSensorVal <1.3)
        {
        	enableButtonWithLabel(buttonDischarging);
        }else if (voltageSensorVal >2.1 && voltageSensorVal <2.5)
        {
        	enableButtonWithLabel(buttonCharging);
        }
    }else
    {
    	enableButtonWithLabel(buttonStandby);
    	//Show idle status of pump
		showPumpRunningStatus(false);
    }

	buttonInit.invalidate();
	buttonStandby.invalidate();
	buttonCharging.invalidate();
	buttonDischarging.invalidate();
}

void Screen1View::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
	if (&src == &toggleButton1_SysOnOff)
	{
	// Toggle system state
	bSystemOn = !bSystemOn;

	if (toggleButton1_SysOnOff.getState())
	{
		// Show buttonInit immediately
		buttonInit.setAlpha(255);
		buttonInit.invalidate();

		// Start 1-second non-blocking timer
		waitTimerActive = true;
		waitCounter = 0;
		lastToggleState = true;
	}
	else
	{
		// Toggle off: reset all buttons & sliders
		buttonInit.setAlpha(150);
		buttonStandby.setAlpha(150);
		buttonCharging.setAlpha(150);
		buttonDischarging.setAlpha(150);

		PumpCurrentSensor_slider.setValue(0);
		VoltageSensor_slider.setValue(0);

		showPumpRunningStatus(false);

		SysStateBgError.setAlpha(0);
		imagePumpError.setAlpha(0);

		buttonInit.invalidate();
		buttonStandby.invalidate();
		buttonCharging.invalidate();
		buttonDischarging.invalidate();

		// Stop any pending timer
		waitTimerActive = false;
	}
}
}

void Screen1View::handleTickEvent()
{
	Screen1ViewBase::handleTickEvent();

	if (waitTimerActive)
	{
		waitCounter++;

		// At 60 FPS, 60 ticks ≈ 1 second
		if (waitCounter >= 60)
		{
			waitTimerActive = false;

			// If button was toggled off during the wait, skip
			if (!lastToggleState) return;

			// Execute the 1-second-later logic
			if (isSystemError())
			{
				enableButtonWithLabel(buttonStandby);

				if (isSystemPumpCurrentError())
				{
					showPumpRunningStatus(false);
					imagePumpError.setAlpha(255);
					imagePumpError.invalidate();
				}

				if (isSystemVoltageError())
				{
					SysStateBgError.setAlpha(255);
					SysStateBgError.invalidate();
				}
			}
			else
			{
				if (isSystemCharging())
					enableButtonWithLabel(buttonCharging);
				else
					enableButtonWithLabel(buttonDischarging);

				showPumpRunningStatus(true);
				imagePumpError.setAlpha(0);
				SysStateBgError.setAlpha(0);
			}

			buttonStandby.invalidate();
			buttonCharging.invalidate();
			buttonDischarging.invalidate();
		}
	}
}

