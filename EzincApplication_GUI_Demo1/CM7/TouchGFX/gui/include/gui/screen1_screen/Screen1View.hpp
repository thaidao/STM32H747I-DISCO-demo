#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void handleTickEvent() override;

    void sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value);
	void sliderValueConfirmedCallbackHandler(const touchgfx::Slider& src, int value);
	void buttonCallbackHandler(const touchgfx::AbstractButton& src);

	void enableButtonWithLabel(touchgfx::ButtonWithLabel &button);
	bool isSystemError();
	bool isSystemVoltageError();
	bool isSystemPumpCurrentError();

	bool isSystemCharging();
	void showPumpRunningStatus(bool bYes);

protected:
	bool bSystemOn = false;

private:
    bool waitTimerActive;       // true when waiting for 1 sec
    uint16_t waitCounter;       // counts ticks (~60 ticks = 1 sec)
    bool lastToggleState;       // store last toggleButton1_SysOnOff state
};

#endif // SCREEN1VIEW_HPP
