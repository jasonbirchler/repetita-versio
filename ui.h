#pragma once

#include "hw.h"
#include "repetita.h"
#include "wreath/head.h"
#include "Utility/dsp.h"
#include <string>

namespace wreath
{
    using namespace daisy;
    using namespace daisysp;
    using namespace patch_sm;

    constexpr float kMaxMsHoldForTrigger{300.f};
    constexpr float kMaxGain{5.f};
    constexpr float kMaxFilterValue{1500.f};
    constexpr float kMaxRateSlew{10.f};

    enum Channel
    {
        LEFT,
        RIGHT,
        BOTH,
        SETTINGS,
    };
    Channel prevChannel{Channel::BOTH};
    Channel currentChannel{Channel::BOTH};

    float channelValues[3][7]{};
    float deltaValues[2][7]{};
    float knobValues[7]{};

    enum TriggerMode
    {
        LOOP,
        REC,
        ONESHOT,
    };
    enum class ButtonHoldMode
    {
        NO_MODE,
        SETTINGS,
        ARM,
    };
    ButtonHoldMode buttonHoldMode{ButtonHoldMode::NO_MODE};
    TriggerMode currentTriggerMode{};
    bool buttonPressed{};
    bool gateTriggered{};
    int32_t buttonHoldStartTime{};
    bool recordingArmed{};
    bool recordingLeftTriggered{};
    bool recordingRightTriggered{};

    bool startUp{true};
    bool first{true};
    bool buffering{};

    struct Settings
    {
        float inputGain;
        float filterType;
        float loopSync;
        float filterLevel;
        float rateSlew;
        float stereoWidth;
        float degradation;
    };

    Settings defaultSettings{1.f / kMaxGain, 0.5f, 0.f, 0.5f, 0.f, 1.f, 0.f};
    Settings localSettings{};

    PersistentStorage<Settings> storage(hw.qspi);

    bool mustUpdateStorage{};

    bool operator!=(const Settings& lhs, const Settings& rhs)
    {
        return lhs.inputGain != rhs.inputGain || lhs.filterType != rhs.filterType || lhs.loopSync != rhs.loopSync || lhs.filterLevel != rhs.filterLevel || lhs.rateSlew != rhs.rateSlew || lhs.stereoWidth != rhs.stereoWidth || lhs.degradation != rhs.degradation;
    }

    inline void ClearLeds()
    {
        hw.SetLed(0);
    }

    float Map(float value, float aMin, float aMax, float bMin, float bMax)
    {
        float k = std::abs(bMax - bMin) / std::abs(aMax - aMin) * (bMax > bMin ? 1 : -1);

        return bMin + k * (value - aMin);
    }

    void SettingsMode(bool active)
    {
        if (active)
        {
            prevChannel = currentChannel;
            currentChannel = Channel::SETTINGS;
        }
        else
        {
            currentChannel = prevChannel;
        }
    }

    // 0: center, 1: left, 2: right
    static void HandleChannelSwitch()
    {
        short value = toggle.Pressed() ? 0 : 1;// [0,1,2] - 1 = [-1, 0, 1]
        //value = value < 0 ? 2 : value; // [2, 0, 1]
        if (value == prevChannel)
        {
            return;
        }

        // Quit edit mode when changing.
        if (Channel::SETTINGS == currentChannel)
        {
            SettingsMode(false);
        }

        currentChannel = prevChannel = static_cast<Channel>(value);
    }

    // 0: center, 1: left, 2: right
    static void HandleTriggerSwitch(bool init = false)
    {
        short value = toggle.Pressed() ? 0 : 1;
        if (value == currentTriggerMode && !init)
        {
            return;
        }

        currentTriggerMode = static_cast<TriggerMode>(value);

        switch (currentTriggerMode)
        {
        case TriggerMode::REC:
            looper.mustStopWriting = true;
            looper.SetLooping(true);
            break;
        case TriggerMode::LOOP:
            recordingLeftTriggered = false;
            recordingRightTriggered = false;
            looper.mustStartWritingLeft = true;
            looper.mustStartWritingRight = true;
            looper.mustStartReading = true;
            looper.SetLooping(true);
            break;
        case TriggerMode::ONESHOT:
            looper.mustStopReading = true;
            looper.SetLooping(false);
            break;
        }
    }

    void HandleTriggerRecording()
    {
        if (recordingLeftTriggered)
        {
            if (Channel::BOTH == currentChannel || Channel::LEFT == currentChannel)
            {
                looper.mustStopWritingLeft = true;
            }
            recordingLeftTriggered = false;
        }
        else
        {
            if (Channel::BOTH == currentChannel || Channel::LEFT == currentChannel)
            {
                looper.mustStartWritingLeft = true;
            }
            recordingLeftTriggered = true;
        }
        if (recordingRightTriggered)
        {
            if (Channel::BOTH == currentChannel || Channel::RIGHT == currentChannel)
            {
                looper.mustStopWritingRight = true;
            }
            recordingRightTriggered = false;
        }
        else
        {
            if (Channel::BOTH == currentChannel || Channel::RIGHT == currentChannel)
            {
                looper.mustStartWritingRight = true;
            }
            recordingRightTriggered = true;
        }
    }

    inline void ProcessParameter(short idx, float value, Channel channel)
    {
        // Keep track of parameters values only after startup.
        if (!looper.IsStartingUp())
        {
            if (Channel::SETTINGS != channel)
            {
                channelValues[channel][idx] = value;
            }
        }

        float leftValue{};
        float rightValue{};

        if (Channel::LEFT == channel)
        {
            deltaValues[Channel::LEFT][idx] = value - channelValues[Channel::BOTH][idx];
        }
        else if (Channel::RIGHT == channel)
        {
            deltaValues[Channel::RIGHT][idx] = value - channelValues[Channel::BOTH][idx];
        }

        switch (idx)
        {
        // Blend
        case CV_1:
            if (Channel::SETTINGS == channel)
            {
                localSettings.inputGain = value;
                looper.inputGain = value * kMaxGain;
                mustUpdateStorage = true;
            }
            else
            {
                looper.dryWetMix = value;
            }
            break;
        // Start
        case CV_2:
        {
            if (Channel::SETTINGS == channel)
            {
                localSettings.stereoWidth = looper.stereoWidth = value;
                mustUpdateStorage = true;
            }
            else
            {
                if (Channel::BOTH == channel || Channel::LEFT == channel)
                {
                    float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::LEFT][idx], 0.f, 1.f) : value;
                    leftValue = Map(v, 0.f, 1.f, 0.f, looper.GetBufferSamples(Channel::LEFT) - 1);
                    looper.SetLoopStart(Channel::LEFT, leftValue);
                }
                if (Channel::BOTH == channel || Channel::RIGHT == channel)
                {
                    float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::RIGHT][idx], 0.f, 1.f) : value;
                    rightValue = Map(v, 0.f, 1.f, 0.f, looper.GetBufferSamples(Channel::RIGHT) - 1);
                    looper.SetLoopStart(Channel::RIGHT, rightValue);
                }
            }
        }
        break;
        // Tone
        case CV_3:
            if (Channel::SETTINGS == channel)
            {
                if (value < 0.33f)
                {
                    looper.filterType = StereoLooper::FilterType::LP;
                }
                else if (value >= 0.33f && value <= 0.66f)
                {
                    looper.filterType = StereoLooper::FilterType::BP;
                }
                else
                {
                    looper.filterType = StereoLooper::FilterType::HP;
                }
                localSettings.filterType = value;
                mustUpdateStorage = true;
            }
            else
            {
                looper.SetFilterValue(Map(value, 0.f, 1.f, 0.f, kMaxFilterValue));
            }
            break;
        // Size
        case CV_4:
        {
            if (Channel::SETTINGS == channel)
            {
                localSettings.loopSync = value;
                looper.SetLoopSync(Channel::BOTH, value >= 0.5);
                mustUpdateStorage = true;
            }
            else
            {
                if (Channel::BOTH == channel || Channel::LEFT == channel)
                {
                    float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::LEFT][idx], 0.f, 1.f) : value;

                    // Backwards, from buffer's length to 50ms.
                    if (v <= 0.35f)
                    {
                        looper.SetLoopLength(Channel::LEFT, Map(v, 0.f, 0.35f, looper.GetBufferSamples(Channel::LEFT), kMinSamplesForFlanger));
                        looper.SetDirection(Channel::LEFT, Direction::BACKWARDS);
                    }
                    // Backwards, from 50ms to 1ms (grains).
                    else if (v < 0.47f)
                    {
                        looper.SetLoopLength(Channel::LEFT, Map(v, 0.35f, 0.47f, kMinSamplesForFlanger, kMinSamplesForTone));
                        looper.SetDirection(Channel::LEFT, Direction::BACKWARDS);
                    }
                    // Forward, from 1ms to 50ms (grains).
                    else if (v >= 0.53f && v < 0.65f)
                    {
                        looper.SetLoopLength(Channel::LEFT, Map(v, 0.53f, 0.65f, kMinSamplesForTone, kMinSamplesForFlanger));
                        looper.SetDirection(Channel::LEFT, Direction::FORWARD);
                    }
                    // Forward, from 50ms to buffer's length.
                    else if (v >= 0.65f)
                    {
                        looper.SetLoopLength(Channel::LEFT, Map(v, 0.65f, 1.f, kMinSamplesForFlanger, looper.GetBufferSamples(Channel::LEFT)));
                        looper.SetDirection(Channel::LEFT, Direction::FORWARD);
                    }
                    // Center dead zone.
                    else
                    {
                        looper.SetLoopLength(Channel::LEFT, Map(v, 0.47f, 0.53f, kMinLoopLengthSamples, kMinLoopLengthSamples));
                        looper.SetDirection(Channel::LEFT, Direction::FORWARD);
                    }

                    // Refresh the rate parameter if the note mode changed.
                    // static StereoLooper::NoteMode noteModeLeft{};
                    // if (noteModeLeft != looper.noteModeLeft)
                    // {
                    //     ProcessParameter(DaisyVersio::KNOB_5, knobValues[DaisyVersio::KNOB_5], Channel::LEFT);
                    // }
                    // noteModeLeft = looper.noteModeLeft;
                }

                if (Channel::BOTH == channel || Channel::RIGHT == channel)
                {
                    float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::RIGHT][idx], 0.f, 1.f) : value;

                    // Backwards, from buffer's length to 50ms.
                    if (v <= 0.35f)
                    {
                        looper.SetLoopLength(Channel::RIGHT, Map(v, 0.f, 0.35f, looper.GetBufferSamples(Channel::RIGHT), kMinSamplesForFlanger));
                        looper.SetDirection(Channel::RIGHT, Direction::BACKWARDS);
                    }
                    // Backwards, from 50ms to 1ms (grains).
                    else if (v < 0.47f)
                    {
                        looper.SetLoopLength(Channel::RIGHT, Map(v, 0.35f, 0.47f, kMinSamplesForFlanger, kMinSamplesForTone));
                        looper.SetDirection(Channel::RIGHT, Direction::BACKWARDS);
                    }
                    // Forward, from 1ms to 50ms (grains).
                    else if (v >= 0.53f && v < 0.65f)
                    {
                        looper.SetLoopLength(Channel::RIGHT, Map(v, 0.53f, 0.65f, kMinSamplesForTone, kMinSamplesForFlanger));
                        looper.SetDirection(Channel::RIGHT, Direction::FORWARD);
                    }
                    // Forward, from 50ms to buffer's length.
                    else if (v >= 0.65f)
                    {
                        looper.SetLoopLength(Channel::RIGHT, Map(v, 0.65f, 1.f, kMinSamplesForFlanger, looper.GetBufferSamples(Channel::RIGHT)));
                        looper.SetDirection(Channel::RIGHT, Direction::FORWARD);
                    }
                    // Center dead zone.
                    else
                    {
                        looper.SetLoopLength(Channel::RIGHT, Map(v, 0.47f, 0.53f, kMinLoopLengthSamples, kMinLoopLengthSamples));
                        looper.SetDirection(Channel::RIGHT, Direction::FORWARD);
                    }

                    // Refresh the rate parameter if the note mode changed.
                    // static StereoLooper::NoteMode noteModeRight{};
                    // if (noteModeRight != looper.noteModeRight)
                    // {
                    //     ProcessParameter(DaisyVersio::KNOB_5, knobValues[DaisyVersio::KNOB_5], Channel::RIGHT);
                    // }
                    // noteModeRight = looper.noteModeRight;
                }
            }
        }
        break;
        // Decay
        // case DaisyVersio::KNOB_4:
        //     if (Channel::SETTINGS == channel)
        //     {
        //         localSettings.filterLevel = looper.filterLevel = value;
        //         mustUpdateStorage = true;
        //     }
        //     else
        //     {
        //         looper.feedback = value;
        //     }
        //     break;
        // Rate
        // case DaisyVersio::KNOB_5:
        //     if (Channel::SETTINGS == channel)
        //     {
        //         localSettings.rateSlew = value;
        //         looper.rateSlew = Map(value, 0.f, 1.f, 0.f, kMaxRateSlew);
        //         mustUpdateStorage = true;
        //     }
        //     else
        //     {
        //         if (Channel::BOTH == channel || Channel::LEFT == channel)
        //         {
        //             float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::LEFT][idx], 0.f, 1.f) : value;

        //             if (StereoLooper::NoteMode::NOTE == looper.noteModeLeft)
        //             {
        //                 // In "note" mode, the rate knob sets the pitch, with 4
        //                 // octaves span.
        //                 leftValue = std::floor(Map(v, 0.f, 1.f, -24, 24)) - 24;
        //                 leftValue = std::pow(2.f, leftValue / 12);
        //             }
        //             else if (StereoLooper::NoteMode::FLANGER == looper.noteModeLeft)
        //             {
        //                 // In "note" mode, the rate knob sets the pitch, with 4
        //                 // octaves span.
        //                 leftValue = Map(v, 0.f, 1.f, -24, 24);
        //                 leftValue = std::pow(2.f, leftValue / 12);
        //             }
        //             else
        //             {
        //                 if (v < 0.45f)
        //                 {
        //                     leftValue = Map(v, 0.f, 0.45f, kMinSpeedMult, 1.f);
        //                 }
        //                 else if (v > 0.55f)
        //                 {
        //                     leftValue = Map(v, 0.55f, 1.f, 1.f, kMaxSpeedMult);
        //                 }
        //                 // Center dead zone.
        //                 else
        //                 {
        //                     leftValue = Map(v, 0.45f, 0.55f, 1.f, 1.f);
        //                 }
        //             }
        //             looper.SetReadRate(Channel::LEFT, leftValue);
        //         }

        //         if (Channel::BOTH == channel || Channel::RIGHT == channel)
        //         {
        //             float v = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::RIGHT][idx], 0.f, 1.f) : value;

        //             if (StereoLooper::NoteMode::NOTE == looper.noteModeRight)
        //             {
        //                 // In "note" mode, the rate knob sets the pitch, with 4
        //                 // octaves span.
        //                 rightValue = std::floor(Map(v, 0.f, 1.f, -24, 24)) - 24;
        //                 rightValue = std::pow(2.f, rightValue / 12);
        //             }
        //             else if (StereoLooper::NoteMode::FLANGER == looper.noteModeRight)
        //             {
        //                 // In "note" mode, the rate knob sets the pitch, with 4
        //                 // octaves span.
        //                 rightValue = Map(v, 0.f, 1.f, -24, 24);
        //                 rightValue = std::pow(2.f, rightValue / 12);
        //             }
        //             else
        //             {
        //                 if (v < 0.45f)
        //                 {
        //                     rightValue = Map(v, 0.f, 0.45f, kMinSpeedMult, 1.f);
        //                 }
        //                 else if (v > 0.55f)
        //                 {
        //                     rightValue = Map(v, 0.55f, 1.f, 1.f, kMaxSpeedMult);
        //                 }
        //                 // Center dead zone.
        //                 else
        //                 {
        //                     rightValue = Map(v, 0.45f, 0.55f, 1.f, 1.f);
        //                 }
        //             }

        //             looper.SetReadRate(Channel::RIGHT, rightValue);
        //         }
        //     }
        //     break;
        // Freeze
        // case DaisyVersio::KNOB_6:
        //     if (Channel::SETTINGS == channel)
        //     {
        //         localSettings.degradation = value;
        //         looper.SetDegradation(value);
        //         mustUpdateStorage = true;
        //     }
        //     else
        //     {
        //         if (Channel::BOTH == channel || Channel::LEFT == channel)
        //         {
        //             float leftValue = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::LEFT][idx], 0.f, 1.f) : value;
        //             looper.SetFreeze(Channel::LEFT, leftValue);
        //         }
        //         if (Channel::BOTH == channel || Channel::RIGHT == channel)
        //         {
        //             float rightValue = (Channel::BOTH == channel) ? fclamp(value + deltaValues[Channel::RIGHT][idx], 0.f, 1.f) : value;
        //             looper.SetFreeze(Channel::RIGHT, rightValue);
        //         }
        //     }
        //     break;

        default:
            break;
        }
    }

    inline void ProcessKnob(int idx)
    {
        float value = knobs[idx].Process();
        // Handle range limits.
        if (value < kMinValueDelta)
        {
            value = 0.f;
        }
        else if (value > 1 - kMinValueDelta)
        {
            value = 1.f;
        }

        // Process the parameter only if it actually changed.
        if (std::abs(knobValues[idx] - value) > kMinValueDelta)
        {
            ProcessParameter(idx, value, currentChannel);

            knobValues[idx] = value;
        }
    }

    inline void ProcessUi()
    {
        if (looper.IsStartingUp())
        {
            if (startUp)
            {
                // TODO: Restore defaults when holding the button during startup.
                // storage.RestoreDefaults();

                startUp = false;

                // Init the dry/wet mix parameter.
                knobValues[CV_1] = knobs[CV_1].Process();
                ProcessParameter(CV_1, knobValues[CV_1], Channel::BOTH);

                // Init the settings.
                Settings &storedSettings = storage.GetSettings();
                ProcessParameter(CV_1, storedSettings.inputGain, Channel::SETTINGS);
                ProcessParameter(CV_2, storedSettings.stereoWidth, Channel::SETTINGS);
                ProcessParameter(CV_3, storedSettings.filterType, Channel::SETTINGS);
                ProcessParameter(CV_4, storedSettings.loopSync, Channel::SETTINGS);
                // ProcessParameter(DaisyVersio::KNOB_4, storedSettings.filterLevel, Channel::SETTINGS);
                // ProcessParameter(DaisyVersio::KNOB_5, storedSettings.rateSlew, Channel::SETTINGS);
                // ProcessParameter(DaisyVersio::KNOB_6, storedSettings.degradation, Channel::SETTINGS);
            }

            return;
        }

        // The looper is buffering.
        if (looper.IsBuffering())
        {
            buffering = true;

            // Stop buffering.
            if (tap.RisingEdge() || (hw.gate_in_1.Trig() && !first))
            {
                ClearLeds();
                looper.mustStopBuffering = true;
            }

            return;
        }

        // The looper is ready, do some configuration before starting.
        if (looper.IsReady())
        {
            if (!buffering)
            {
                return;
            }
            buffering = false;

            HandleTriggerSwitch(true);

            // Init all the parameters with the relative knobs position.
            for (size_t i = 0; i < 4; i++)
            {
                knobValues[i] = knobs[i].Process();
                if (knobValues[i] < kMinValueDelta)
                {
                    knobValues[i] = 0.f;
                }
                else if (knobValues[i] > 1 - kMinValueDelta)
                {
                    knobValues[i] = 1.f;
                }
                for (short j = 2; j >= 0; j--)
                {
                    channelValues[j][i] = knobValues[i];
                    ProcessParameter(i, knobValues[i], static_cast<Channel>(j));
                }
            }

            looper.Start();

            return;
        }

        // At this point the looper is running, do the normal UI loop.

        HandleTriggerSwitch();
        HandleChannelSwitch();

        ProcessKnob(CV_4); // Size
        ProcessKnob(CV_2); // Start
        // ProcessKnob(DaisyVersio::KNOB_6); // Freeze

        ProcessKnob(CV_1); // Blend
        ProcessKnob(CV_3); // Tone
        // ProcessKnob(DaisyVersio::KNOB_4); // Decay
        // ProcessKnob(DaisyVersio::KNOB_5); // Rate

        // Handle button press.
        if (tap.RisingEdge() && !buttonPressed)
        {
            buttonPressed = true;
            buttonHoldStartTime = System::GetNow();
        }

        // Handle button release.
        if (tap.FallingEdge() && buttonPressed)
        {
            buttonPressed = false;
            if (ButtonHoldMode::SETTINGS == buttonHoldMode)
            {
                SettingsMode(true);
                buttonHoldMode = ButtonHoldMode::NO_MODE;
            }
            else if (ButtonHoldMode::ARM == buttonHoldMode)
            {
                recordingArmed = true;
                buttonHoldMode = ButtonHoldMode::NO_MODE;
            }
            else
            {
                if (recordingArmed)
                {
                    looper.mustResetLooper = true;
                    recordingArmed = false;
                }
                else if (System::GetNow() - buttonHoldStartTime <= kMaxMsHoldForTrigger)
                {
                    if (Channel::SETTINGS == currentChannel)
                    {
                        SettingsMode(false);
                    }
                    else
                    {
                        if (TriggerMode::ONESHOT == currentTriggerMode)
                        {
                            looper.mustRestart = true;
                        }
                        else if (TriggerMode::REC == currentTriggerMode)
                        {
                            HandleTriggerRecording();
                        }
                        else
                        {
                            looper.mustRetrigger = true;
                        }
                    }
                }
            }
        }

        // Do something while the button is pressed.
        if (buttonPressed && !recordingLeftTriggered && !recordingRightTriggered)
        {
            if (recordingArmed)
            {
                if (System::GetNow() - buttonHoldStartTime > 1000.f)
                {
                    recordingArmed = false;
                    buttonPressed = false;
                }
            }
            else
            {
                if (System::GetNow() - buttonHoldStartTime > kMaxMsHoldForTrigger)
                {
                    buttonHoldMode = ButtonHoldMode::SETTINGS;
                }
                if (System::GetNow() - buttonHoldStartTime > 1500.f)
                {
                    if (Channel::SETTINGS == currentChannel)
                    {
                        SettingsMode(false);
                    }
                    buttonHoldMode = ButtonHoldMode::ARM;
                }
            }
        }

        if (Channel::SETTINGS != currentChannel && ButtonHoldMode::NO_MODE == buttonHoldMode && !first)
        {
            if (hw.gate_in_1.Trig())
            {
                if (recordingArmed)
                {
                    SettingsMode(false);
                    looper.mustResetLooper = true;
                    recordingArmed = false;
                }
                else if (TriggerMode::REC == currentTriggerMode)
                {
                    HandleTriggerRecording();
                }
                else
                {
                    if (TriggerMode::ONESHOT == currentTriggerMode)
                    {
                        looper.mustRestart = true;
                    }
                    else
                    {
                        looper.mustRetrigger = true;
                    }
                }
            }
        }

        first = false;
    }

    inline void InitUi()
    {
        storage.Init(defaultSettings);
    }

    void ProcessStorage()
    {
        if (mustUpdateStorage)
        {
            if (!looper.IsStartingUp())
            {
                Settings &storedSettings = storage.GetSettings();
                storedSettings = localSettings;
                storage.Save();
            }
            mustUpdateStorage = false;
        }
    }
}