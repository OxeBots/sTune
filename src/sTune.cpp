/****************************************************************************************
   sTune Library for Arduino - Version 2.4.0
   by dlloydev https://github.com/Dlloydev/sTune
   Licensed under the MIT License.

  This is an open loop PID autotuner using a novel s-curve inflection point test method.
  Tuning parameters are determined in about ½Tau on a first-order system with time delay.
  Full 5Tau testing and multiple serial output options are provided.
 ****************************************************************************************/

#include "sTune.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sTan.h"

#define micros() (uint32_t)esp_timer_get_time()
#define millis() (uint32_t)(esp_timer_get_time() / 1000)

#ifndef HIGH
#define HIGH 1
#endif
#ifndef LOW
#define LOW 0
#endif

static const char * TAG = "sTune";

sTan::sTan()
{
}
sTan tangent = sTan();

sTune::sTune()
{
    _input = nullptr;
    _output = nullptr;
    sTune::Reset();
}

sTune::sTune(float * input, float * output, TuningMethod tuningMethod, Action action, SerialMode serialMode)
{
    _input = input;
    _output = output;
    _tuningMethod = tuningMethod;
    _action = action;
    _serialMode = serialMode;
    sTune::Reset();
}

void sTune::Reset()
{
    _tunerStatus = test;
    *_output = _outputStart;
    usPrev = micros();
    settlePrev = usPrev;
    ipUs = 0;
    us = 0;
    _Ku = 0.0f;
    _Tu = 0.0f;
    _td = 0.0f;
    _kp = 0.0f;
    _ki = 0.0f;
    _kd = 0.0f;
    pvIp = 0.0f;
    pvMax = 0.0f;
    pvPk = 0.0f;
    slopeIp = 0.0f;
    pvTangent = 0.0f;
    pvTangentPrev = 0.0f;
    pvAvg = pvInst;
    pvStart = pvInst;
    pvInstRes = pvInst;
    ipCount = 0;
    plotCount = 0;
    sampleCount = 0;
    pvPkCount = 0;
}

void sTune::Configure(const float inputSpan, const float outputSpan, float outputStart, float outputStep,
                      uint32_t testTimeSec, uint32_t settleTimeSec, const uint16_t samples)
{
    sTune::Reset();
    _inputSpan = inputSpan;
    eStop = inputSpan;
    _outputSpan = outputSpan;
    _outputStart = outputStart;
    _outputStep = outputStep;
    _testTimeSec = testTimeSec;
    _settleTimeSec = settleTimeSec;
    _samples = samples;
    _bufferSize = (uint16_t)(_samples * 0.06);
    _samplePeriodUs = (float)(_testTimeSec * 1000000.0f) / _samples;
    _tangentPeriodUs = _samplePeriodUs * (_bufferSize - 1);
    _settlePeriodUs = (float)(_settleTimeSec * 1000000.0f);
    tangent.begin(_bufferSize);
}

uint8_t sTune::Run()
{
    uint32_t usNow = micros();
    uint32_t usElapsed = usNow - usPrev;
    uint32_t settleElapsed = usNow - settlePrev;
    us = usNow - usStart;

    switch (_tunerStatus)
    {
        case sample:
            _tunerStatus = test;
            return test;
            break;

        case test:  // run inflection point test method
            if (pvInst > eStop && !eStopAbort)
            {
                sTune::Reset();
                sampleCount = _samples + 1;
                eStopAbort = 1;
                ESP_LOGW(TAG, "ABORT: pvInst > eStop");
                break;
            }
            if (settleElapsed >= _settlePeriodUs)
            {  // if settling period has expired
                if (sampleCount == 1)
                    *_output = _outputStep;
                if (usElapsed >= _samplePeriodUs)
                {  // ready to process a sample
                    usPrev = usNow;
                    if (sampleCount <= _samples)
                    {  // continue testing

                        // get pvInst and pvAvg (buffer) resolution
                        float lastPvInst = pvInst;
                        float lastPvAvg = pvAvg;
                        pvInst = *_input;
                        pvAvg = tangent.avgVal(pvInst);
                        float pvInstResolution = fabs(pvInst - lastPvInst);
                        float pvAvgResolution = fabs(pvAvg - lastPvAvg);
                        if (pvInstResolution > epsilon && pvInstResolution < pvInstRes)
                            pvInstRes = pvInstResolution;
                        if (pvAvgResolution > epsilon && pvAvgResolution < pvAvgRes)
                            pvAvgRes = pvAvgResolution;

                        if (sampleCount == 0)
                        {  // initialize at first sample
                            tangent.init(pvInst);
                            pvAvg = pvInst;
                            pvInstResolution = 0.0f;
                            pvAvgResolution = 0.0f;
                            pvInstRes = pvInst;
                            pvAvgRes = pvInst;
                            pvStart = pvInst;
                            usStart = usNow;
                            us = 0;
                        }

                        // To determine the point of inflection, we'll use a sliding tangent line.
                        // https://en.wikipedia.org/wiki/Inflection_point#/media/File:Animated_illustration_of_inflection_point.gif
                        pvTangent = pvAvg - tangent.startVal();

                        // check for dead time
                        bool dt = false;
                        if (_action == directIP || _action == direct5T)
                        {
                            (pvAvg > pvStart + pvInstRes + epsilon) ? dt = true : dt = false;
                        }
                        else
                        {  // reverse
                            (pvAvg < pvStart - pvInstRes - epsilon) ? dt = true : dt = false;
                        }
                        if (!_td && dt)
                            _td = us * 0.000001f;

                        // check for inflection point
                        bool ipcount = false;
                        if (_action == directIP || _action == direct5T)
                        {
                            if (pvTangent > slopeIp + epsilon)
                                ipcount = true;
                            if (pvTangent < 0 + epsilon)
                                ipCount = 0;  // flat or negative tangent
                        }
                        else
                        {  // reverse
                            if (pvTangent < slopeIp - epsilon)
                                ipcount = true;
                            if (pvTangent > 0 - epsilon)
                                ipCount = 0;  // flat or positive tangent
                        }
                        if (ipcount)
                        {
                            ipCount = 0;
                            slopeIp = pvTangent;
                        }
                        ipCount++;
                        if ((_action == directIP || _action == reverseIP) && (ipCount == ((uint16_t)(_samples / 16))))
                        {  // reached inflection point
                            sampleCount = _samples;
                            ipUs = us;
                            pvIp = pvAvg;

                            // apparent pvMax  Re: "Step response with arbitrary initial conditions"
                            // https://en.wikipedia.org/wiki/Time_constant
                            pvMax = pvIp + (slopeIp * kexp);

                            // apparent tangent from pvStart to pvMax crossing points
                            _Tu = (((pvMax - pvStart) / slopeIp) * _tangentPeriodUs * 0.000001f) - _td;
                        }

                        if (_action == direct5T || _action == reverse5T)
                        {  // continue testing to maximum input
                            if (sampleCount >= _samples - 1)
                                sampleCount = _samples - 2;
                            if (us > _testTimeSec * 100000)
                            {  // 10% of testTimeSec has elapsed
                                if (pvAvg > pvPk)
                                {
                                    pvPk = pvAvg + (_bufferSize * 0.2f * pvAvgRes);  // set a new boosted peak
                                    pvPkCount = 0;                                   // reset the "below peak" counter
                                }
                                else
                                {
                                    pvPkCount++;  // count up while pvAvg is below the boosted peak
                                }
                                if (pvPkCount == ((uint16_t)(1.2 * _bufferSize)))
                                {  // test done
                                    pvPkCount++;
                                    sampleCount = _samples;
                                    pvMax = pvAvg + (pvInst - pvStart) * 0.05f;  // assume 3τ, so increase pvMax by 5%
                                    _Tu = (us * 1.6667 * 0.000001f * 0.286f) -
                                          _td;  // scale us to 5τ in seconds, then multiply by 0.286 for τ
                                }
                            }
                        }

                        if (sampleCount == _samples)
                        {  // testing complete
                            _R = _td / _Tu;

                            // process gain
                            _Ku = fabs(((pvMax - pvStart) / _inputSpan) / ((_outputStep - _outputStart) / _outputSpan));

                            _kp = sTune::GetKp();
                            _ki = sTune::GetKi();
                            _kd = sTune::GetKd();

                            sTune::printResults();
                            _tunerStatus = tunings;
                            return tunings;
                            break;
                        }
                        sTune::printTestRun();
                        pvTangentPrev = pvTangent;
                    }
                    else
                        _tunerStatus = tunings;
                    sampleCount++;
                    _tunerStatus = sample;
                    return sample;
                }
            }
            else
            {  // settling
                if (usElapsed >= _samplePeriodUs && !eStopAbort)
                {
                    *_output = _outputStart;
                    usPrev = usNow;
                    pvInst = *_input;
                    if (_serialMode == printALL || _serialMode == printDEBUG)
                    {
                        ESP_LOGI(TAG, " sec: %.4f  out: %.2f  pv: %.3f  settling  ⤳⤳",
                                 (float)((_settlePeriodUs - settleElapsed) * 0.000001f), *_output, pvInst);
                    }
                    _tunerStatus = sample;
                    return sample;
                }
            }
            break;

        case tunings:
            _tunerStatus = timerPid;
            return timerPid;
            break;

        case runPid:
            if (pvInst > eStop && !eStopAbort)
            {
                sTune::Reset();
                sampleCount = _samples + 1;
                eStopAbort = 1;
                ESP_LOGW(TAG, "ABORT: pvInst > eStop");
            }
            _tunerStatus = timerPid;
            return timerPid;
            break;

        case timerPid:
            if (usElapsed >= _samplePeriodUs)
            {
                usPrev = usNow;
                _tunerStatus = runPid;
                return runPid;
            }
            else
            {
                _tunerStatus = timerPid;
                return timerPid;
            }
            break;

        default:
            _tunerStatus = timerPid;
            return timerPid;
            break;
    }
    return timerPid;
}

void sTune::SetEmergencyStop(float e_Stop)
{
    eStop = e_Stop;
}

void sTune::SetControllerAction(Action Action)
{
    _action = Action;
}

void sTune::SetSerialMode(SerialMode SerialMode)
{
    _serialMode = SerialMode;
}

void sTune::SetTuningMethod(TuningMethod TuningMethod)
{
    _tuningMethod = TuningMethod;
}

void sTune::printPidTuner(uint8_t everyNth)
{
    if (sampleCount < _samples)
    {
        if (plotCount == 0 || plotCount >= everyNth)
        {
            plotCount = 1;
            ESP_LOGI(TAG, "%.4f, %.2f, %.3f", us * 0.000001f, *_output, pvAvg);
        }
        else
            plotCount++;
    }
}

void sTune::plotter(float input, float output, float setpoint, float outputScale, uint8_t everyNth)
{
    if (plotCount >= everyNth)
    {
        plotCount = 1;
        ESP_LOGI(TAG, "Setpoint:%.2f, Input:%.2f, Output:%.2f", setpoint, input, output * outputScale);
    }
    else
        plotCount++;
}

void sTune::printTestRun()
{
    if (sampleCount < _samples)
    {
        if (_serialMode == printALL || _serialMode == printDEBUG)
        {
            char debug_info[128] = {0};
            int offset = 0;
            offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, " sec: %.4f  out: %.2f  pv: %.3f",
                               us * 0.000001f, *_output, pvInst);
            if (_serialMode == printDEBUG && (_action == direct5T || _action == reverse5T))
            {
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset,
                                   "  pvPk: %.3f  pvPkCount: %d  ipCount: %d", pvPk, pvPkCount, ipCount);
            }
            if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP))
            {
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, "  ipCount: %d", ipCount);
            }
            offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, "  tan: %.3f", pvTangent);
            if (pvInst > 0.9f * eStop)
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, " ⚠");
            if (pvTangent - pvTangentPrev > 0 + epsilon)
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, " ↗");
            else if (pvTangent - pvTangentPrev < 0 - epsilon)
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, " ↘");
            else
                offset += snprintf(debug_info + offset, sizeof(debug_info) - offset, " →");
            ESP_LOGI(TAG, "%s", debug_info);
        }
    }
}

void sTune::printTunings()
{
    const char * method_str;
    switch (_tuningMethod)
    {
        case ZN_PID:
            method_str = "ZN_PID";
            break;
        case DampedOsc_PID:
            method_str = "Damped_PID";
            break;
        case NoOvershoot_PID:
            method_str = "NoOvershoot_PID";
            break;
        case CohenCoon_PID:
            method_str = "CohenCoon_PID";
            break;
        case Mixed_PID:
            method_str = "Mixed_PID";
            break;
        case ZN_PI:
            method_str = "ZN_PI";
            break;
        case DampedOsc_PI:
            method_str = "Damped_PI";
            break;
        case NoOvershoot_PI:
            method_str = "NoOvershoot_PI";
            break;
        case CohenCoon_PI:
            method_str = "CohenCoon_PI";
            break;
        default:
            method_str = "Mixed_PI";
            break;
    }
    ESP_LOGI(TAG, " Tuning Method: %s\n  Kp: %.3f\n  Ki: %.3f  Ti: %.3f\n  Kd: %.3f  Td: %.3f", method_str,
             sTune::GetKp(), sTune::GetKi(), sTune::GetTi(), sTune::GetKd(), sTune::GetTd());
}

void sTune::printResults()
{
    if (_serialMode == printALL || _serialMode == printDEBUG || _serialMode == printSUMMARY)
    {
        const char * action_str;
        switch (_action)
        {
            case directIP:
                action_str = "directIP";
                break;
            case direct5T:
                action_str = "direct5T";
                break;
            case reverseIP:
                action_str = "reverseIP";
                break;
            default:
                action_str = "reverse5T";
                break;
        }

        ESP_LOGI(
          TAG, "\n Controller Action: %s\n Output Start:      %.2f\n Output Step:       %.2f\n Sample Sec:        %.4f",
          action_str, _outputStart, _outputStep, _samplePeriodUs * 0.000001f);

        if (_serialMode == printDEBUG && (_action == directIP || _action == reverseIP))
        {
            const char * slope_dir = " ↓";
            if (_action == directIP || _action == direct5T)
                slope_dir = " ↑";
            ESP_LOGI(TAG, " Ip Sec:            %.4f\n Ip Slope:          %.3f%s\n Ip Pv:             %.3f",
                     ipUs * 0.000001f, slopeIp, slope_dir, pvIp);
        }

        const char * pv_label = " Pv Min:            %.3f";
        if (_action == directIP || _action == direct5T)
            pv_label = " Pv Max:            %.3f";
        ESP_LOGI(TAG,
                 " Pv Start:          %.3f\n%s\n Pv Diff:           %.3f\n Process Gain:      %.3f\n Dead Time Sec:    "
                 " %.3f\n Tau Sec:           %.3f",
                 pvStart, pv_label, pvMax, pvMax - pvStart, _Ku, _td, _Tu);

        float controllability = _Tu / _td + epsilon;
        if (controllability > 99.9)
            controllability = 99.9;
        const char * control_quality = " (difficult to control)";
        if (controllability > 0.75)
            control_quality = " (easy to control)";
        else if (controllability > 0.25)
            control_quality = " (average controllability)";
        ESP_LOGI(TAG, " Tau/Dead Time:     %.1f%s", controllability, control_quality);

        // check “best practice” rule that sample time should be ≥ 10 times per process time constant
        // https://controlguru.com/sample-time-is-a-fundamental-design-and-tuning-specification/
        float sampleTimeCheck = _Tu / (_samplePeriodUs * 0.000001f);
        const char * sample_quality = " (low sample rate)";
        if (sampleTimeCheck >= 10)
            sample_quality = " (good sample rate)";
        ESP_LOGI(TAG, " Tau/Sample Period: %.1f%s", sampleTimeCheck, sample_quality);

        sTune::printTunings();
        sampleCount++;
    }
}

// Query functions

void sTune::GetAutoTunings(float * kp, float * ki, float * kd)
{
    *kp = _kp;
    *ki = _ki;
    *kd = _kd;
}

// https://blog.opticontrols.com/archives/477

float sTune::GetKp()
{
    float znPid = ((1.2f * _Tu) / (_Ku * _td)) / 2;
    float doPid = (0.66f * _Tu) / (_Ku * _td);
    float noPid = (0.6f / _Ku) * (_Tu / _td);
    float ccPid = _Ku * (1.33f + (_R / 4.0f));
    float znPi = ((0.9f * _Tu) / (_Ku * _td)) / 2;
    float doPi = (0.495f * _Tu) / (_Ku * _td);
    float noPi = (0.35f / _Ku) * (_Tu / _td);
    float ccPi = _Ku * (0.9f + (_R / 12.0f));
    switch (_tuningMethod)
    {
        case ZN_PID:
            _kp = znPid;
            break;
        case DampedOsc_PID:
            _kp = doPid;
            break;
        case NoOvershoot_PID:
            _kp = noPid;
            break;
        case CohenCoon_PID:
            _kp = ccPid;
            break;
        case Mixed_PID:
            _kp = 0.25f * (znPid + doPid + noPid + ccPid);
            break;
        case ZN_PI:
            _kp = znPi;
            break;
        case DampedOsc_PI:
            _kp = doPi;
            break;
        case NoOvershoot_PI:
            _kp = noPi;
            break;
        case CohenCoon_PI:
            _kp = ccPi;
            break;
        default:
            _kp = 0.25f * (znPi + doPi + noPi + ccPi);
            break;  // Mixed_PI
    }
    return _kp;
}

float sTune::GetKi()
{
    float znPid = 1 / (2.0f * _td);
    float doPid = 1 / (_Tu / 3.6f);
    float noPid = 1 / (_Tu);
    float ccPid = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));
    float znPi = 1 / (3.3333f * _td);
    float doPi = 1 / (_Tu / 2.6f);
    float noPi = 1 / (1.2f * _Tu);
    float ccPi = 1 / (_td * (30.0f + (3.0f * _R)) / (9.0f + (20.0f * _R)));

    switch (_tuningMethod)
    {
        case ZN_PID:
            _ki = znPid;
            break;
        case DampedOsc_PID:
            _ki = doPid;
            break;
        case NoOvershoot_PID:
            _ki = noPid;
            break;
        case CohenCoon_PID:
            _ki = ccPid;
            break;
        case Mixed_PID:
            _ki = 0.25f * (znPid + doPid + noPid + ccPid);
            break;
        case ZN_PI:
            _ki = znPi;
            break;
        case DampedOsc_PI:
            _ki = doPi;
            break;
        case NoOvershoot_PI:
            _ki = noPi;
            break;
        case CohenCoon_PI:
            _ki = ccPi;
            break;
        default:
            _ki = 0.25f * (znPi + doPi + noPi + ccPi);
            break;  // Mixed_PI
    }
    return _ki;
}

float sTune::GetKd()
{
    float znPid = 1 / (0.5f * _td);
    float doPid = 1 / (_Tu / 9.0f);
    float noPid = 1 / (0.5f * _td);
    float ccPid = 1 / ((4.0f * _td) / (11.0f + (2.0f * _R)));
    switch (_tuningMethod)
    {
        case ZN_PID:
            _kd = znPid;
            break;
        case DampedOsc_PID:
            _kd = doPid;
            break;
        case NoOvershoot_PID:
            _kd = noPid;
            break;
        case CohenCoon_PID:
            _kd = ccPid;
            break;
        case Mixed_PID:
            _kd = 0.25f * (znPid + doPid + noPid + ccPid);
            break;
        default:
            _kd = 0.0f;
            break;  // PI controller
    }
    return _kd;
}

float sTune::GetTi()
{
    return _kp / _ki;
}

float sTune::GetTd()
{
    if (_tuningMethod == ZN_PID || _tuningMethod == DampedOsc_PID || _tuningMethod == NoOvershoot_PID ||
        _tuningMethod == CohenCoon_PID || _tuningMethod == Mixed_PID)
    {
        return _kp / _kd;
    }

    return 0.0f;
}

float sTune::GetProcessGain()
{
    return _Ku;
}

float sTune::GetDeadTime()
{
    return _td;
}

float sTune::GetTau()
{
    return _Tu;
}

uint8_t sTune::GetControllerAction()
{
    return static_cast<uint8_t>(_action);
}

uint8_t sTune::GetSerialMode()
{
    return static_cast<uint8_t>(_serialMode);
}

uint8_t sTune::GetTuningMethod()
{
    return static_cast<uint8_t>(_tuningMethod);
}

float sTune::softPwm(const uint8_t relayPin, float input, float output, float setpoint, uint32_t windowSize,
                     uint8_t debounce)
{
    // software PWM timer
    uint32_t msNow = millis();
    static uint32_t windowStartTime, nextSwitchTime;
    if (msNow - windowStartTime >= windowSize)
    {
        windowStartTime = msNow;
    }
    // SSR optimum AC half-cycle controller
    static float optimumOutput;
    static bool reachedSetpoint;

    if (input > setpoint)
        reachedSetpoint = true;
    if (reachedSetpoint && !debounce && setpoint > 0 && input > setpoint)
        optimumOutput = output - 8;
    else if (reachedSetpoint && !debounce && setpoint > 0 && input < setpoint)
        optimumOutput = output + 8;
    else
        optimumOutput = output;
    if (optimumOutput < 0)
        optimumOutput = 0;

    // PWM relay output
    static bool relayStatus;
    if (!relayStatus && optimumOutput > (msNow - windowStartTime))
    {
        if (msNow > nextSwitchTime)
        {
            nextSwitchTime = msNow + debounce;
            relayStatus = true;
            gpio_set_level((gpio_num_t)relayPin, 1);
        }
    }
    else if (relayStatus && optimumOutput < (msNow - windowStartTime))
    {
        if (msNow > nextSwitchTime)
        {
            nextSwitchTime = msNow + debounce;
            relayStatus = false;
            gpio_set_level((gpio_num_t)relayPin, 0);
        }
    }
    return optimumOutput;
}
