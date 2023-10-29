#ifndef TASKRUNDIAGNOSTICS_H
#define TASKRUNDIAGNOSTICS_H

void FloatToUint8s(float* src, uint8_t* array, int position);

void Uint16ToUint8s(uint16_t* src, uint8_t* array, int position);

void Int16ToUint8s(int16_t* src, uint8_t* array, int position);

void TaskDiagnostics(void const *argument);

#endif /* TASKRUNDIAGNOSTICS_H */
