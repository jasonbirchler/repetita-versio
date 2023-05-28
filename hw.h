#pragma once

#include "daisy_patch_sm.h"

namespace wreath
{
    using namespace daisy;
    using namespace patch_sm;

    // The minimum difference in parameter value to be registered.
    constexpr float kMinValueDelta{0.003f};
    // The minimum difference in parameter value to be considered picked up.
    constexpr float kMinPickupValueDelta{0.01f};
    // The trigger threshold value.
    constexpr float kTriggerThres{0.3f};

    DaisyPatchSM hw;

    Parameter knobs[4]{};
    Led led;
    Switch tap, toggle;

    inline void InitHw()
    {
        hw.Init();
        hw.StartAdc();
        led.Init(hw.user_led.pin, false, hw.AudioSampleRate());
        tap.Init(DaisyPatchSM::B7, hw.AudioCallbackRate());
        toggle.Init(DaisyPatchSM::B8, hw.AudioCallbackRate());

        for (short i = 0; i < 4; i++)
        {
            hw.controls[i].SetCoeff(1.f); // No slew;
            knobs[i].Init(hw.controls[i], 0.0f, 1.0f, Parameter::LINEAR);
        }
    }

    inline void ProcessControls()
    {
        hw.ProcessAllControls();
        tap.Debounce();
        toggle.Debounce();
        led.Update();
    }
}