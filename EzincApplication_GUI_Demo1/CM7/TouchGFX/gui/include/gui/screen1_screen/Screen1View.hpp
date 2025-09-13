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

    void sliderValueChangedCallbackHandler(const touchgfx::Slider& src, int value);
	void sliderValueConfirmedCallbackHandler(const touchgfx::Slider& src, int value);
	void buttonCallbackHandler(const touchgfx::AbstractButton& src);

	void enableButtonWithLabel(touchgfx::ButtonWithLabel &button);
	bool isSystemError();
	bool isSystemCharging();
	void showPumpRunningStatus(bool bYes);

protected:
	bool bSystemOn = false;
};

#endif // SCREEN1VIEW_HPP
