#include "Globals.h"

void TaskPower(void const *argument)
{
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 1; //Hz
	const TickType_t xTickDuration = (1000 * 1 / xFrequency) / portTICK_PERIOD_MS; // Ticks to delay the task for

	// ...

	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for (;;)
	{
		// Wait for the next cycle.
		vTaskDelayUntil(&xLastWakeTime, xTickDuration);

		TickType_t time = xTaskGetTickCount();

		//...

		//LogN(xTaskGetTickCount() - time);
	}
}
