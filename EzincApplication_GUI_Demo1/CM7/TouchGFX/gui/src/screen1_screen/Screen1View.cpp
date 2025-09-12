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
    if ((SysStateBgError.getAlpha()== 0) && (imagePumpError.getAlpha() == 0))
        return false;

    return true;
}

void Screen1View::sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value)
{
	//touchgfx_printf("Change value: %d\n", value);

    if (&src == &PumpCurrentSensor_slider)
    {
        //PumpCurrentSensor_ChangeValue
        //When PumpCurrentSensor_slider value changed execute C++ code
        //Execute C++ code
        float currentSensorVal = (float)PumpCurrentSensor_slider.getValue() / 100.0f;

        Unicode::snprintfFloat(
            textArea_CurrentSensorBuffer,
            TEXTAREA_CURRENTSENSOR_SIZE,
            "%.1f",
            currentSensorVal
        );

        //Error
        if(currentSensorVal<1 || currentSensorVal >1.3)
        {
        	imagePumpError.setAlpha(255);
        	enableButtonWithLabel(buttonStandby);
        }
        else
        {
        	imagePumpError.setAlpha(0);

            float voltageSensorVal =     (float)VoltageSensor_slider.getValue() / 100.0f;

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

            if(isSystemError() == false)
            	enableButtonWithLabel(buttonDischarging);
        }
        else if (voltageSensorVal >2.1 && voltageSensorVal <2.5)
		{
        	SysStateBgError.setAlpha(0);

            if(isSystemError() == false)
            	enableButtonWithLabel(buttonCharging);
		}else
        {
        	SysStateBgError.setAlpha(255);
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

    //Update state
    if(isSystemError() == false)
    {
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
    }
}

