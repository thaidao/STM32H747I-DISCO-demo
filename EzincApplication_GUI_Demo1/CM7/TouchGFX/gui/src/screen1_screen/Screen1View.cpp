#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
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
}

bool Screen1View::isSystemError()
{
    float currentSensorVal = (float)PumpCurrentSensor_slider.getValue() / 100.0f;
    float voltageSensorVal = (float)VoltageSensor_slider.getValue() / 100.0f;

//    if ((SysStateBgError.getAlpha()== 0) && (imagePumpError.getAlpha() == 0)){
//        return false;

    if(currentSensorVal<1 || currentSensorVal >1.3){

    	return true;
    }else if(!((voltageSensorVal >0.9 && voltageSensorVal <1.3)|| (voltageSensorVal >2.1 && voltageSensorVal <2.5))){

    	return true;
    }

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
	//Bt_SysOnOff_Clicked
	//When toggleButton1_SysOnOff clicked execute C++ code
	//Execute C++ code
	if (&src == &toggleButton1_SysOnOff)
	{
		//Update on off status
		bSystemOn = bSystemOn == true? false:true;

		if(toggleButton1_SysOnOff.getState()){

			//Bt_SysOnOff_Clicked
			//When toggleButton1_SysOnOff clicked fade buttonInit
			//Set alpha to 255 on buttonInit
//			buttonInit.setAlpha(255);
//			buttonStandby.setAlpha(255);
//			buttonInit.setAlpha(150);

			enableButtonWithLabel(buttonInit);

			//@TODO start timer 2 sec

//			if(isSystemError() == true)
//			{//standby
//				enableButtonWithLabel(buttonStandby);
//			}
//			else
//			{
//				if(isSystemCharging() == true){
//					enableButtonWithLabel(buttonCharging);
//				}else{
//					enableButtonWithLabel(buttonDischarging);
//				}
//			}

		}else
		{
			buttonInit.setAlpha(150);
			buttonStandby.setAlpha(150);
			buttonCharging.setAlpha(150);
			buttonDischarging.setAlpha(150);

			PumpCurrentSensor_slider.setValue(0);
			VoltageSensor_slider.setValue(0);

			//Pump status text area
			showPumpRunningStatus(false);

			//Error Background image
			SysStateBgError.setAlpha(0);
			imagePumpError.setAlpha(0);
		}

		buttonInit.invalidate();
		buttonStandby.invalidate();
		buttonCharging.invalidate();
		buttonDischarging.invalidate();
	}
}

