#define SIZEOF(x) sizeof(x)/sizeof(x[0])

#include "FFBDescriptor.h"

int AppendArray(uint8_t *target, uint8_t *source, uint16_t offset, uint16_t len)
{
    uint8_t *arr = &target[offset];
    memcpy(arr, source, len);
    return offset + len;
}

int GeneratePidReport(uint8_t *pidReport, bool force_x, bool force_y, bool force_z)
{
    uint16_t len = 0;
    uint8_t *tempReport;
    tempReport = new uint8_t[1536];
 
    const uint8_t usagePagePhysicalInterface[] PROGMEM = {0x05, 0x0F};
    len = AppendArray(tempReport, usagePagePhysicalInterface, len, SIZEOF(usagePagePhysicalInterface));
    len = AppendArray(tempReport, stateReport, len, SIZEOF(stateReport));
    len = AppendArray(tempReport, setEffectReport, len, SIZEOF(setEffectReport));
    len = AppendArray(tempReport, setEnvelopeReport, len, SIZEOF(setEnvelopeReport));
    len = AppendArray(tempReport, setConditionReport, len, SIZEOF(setConditionReport));
    len = AppendArray(tempReport, setPeriodicReport, len, SIZEOF(setPeriodicReport));
    len = AppendArray(tempReport, setConstantForceReport, len, SIZEOF(setConstantForceReport));
    len = AppendArray(tempReport, setRampForceReport, len, SIZEOF(setRampForceReport));
    len = AppendArray(tempReport, customForceDataReport, len, SIZEOF(customForceDataReport));
    len = AppendArray(tempReport, downloadForceSample, len, SIZEOF(downloadForceSample));
    len = AppendArray(tempReport, effectOperationReport, len, SIZEOF(effectOperationReport));
    len = AppendArray(tempReport, pidBlockFreeReport, len, SIZEOF(pidBlockFreeReport));
    len = AppendArray(tempReport, pidDeviceControl, len, SIZEOF(pidDeviceControl));
    len = AppendArray(tempReport, deviceGainReport, len, SIZEOF(deviceGainReport));
    len = AppendArray(tempReport, setCustomForceReport, len, SIZEOF(setCustomForceReport));
    len = AppendArray(tempReport, createNewEffectReport, len, SIZEOF(createNewEffectReport));
    len = AppendArray(tempReport, pidBlockLoadReport, len, SIZEOF(pidBlockLoadReport));
    len = AppendArray(tempReport, pidPoolReport, len, SIZEOF(pidPoolReport));
    tempReport[len++] = 0xC0;
    pidReport = new uint8_t[len];
    memcpy(pidReport, tempReport, len);
    delete tempReport;
    return len+1;
}
