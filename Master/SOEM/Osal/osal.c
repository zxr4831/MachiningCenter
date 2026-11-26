/*
 * Licensed under the GNU General Public License version 2 with exceptions. See
 * LICENSE file in the project root for full license information
 */

#include <stdlib.h>
#include <stdbool.h>
#include "osal.h"
#include "tim.h"


/* Workaround for rt-labs defect 776.
 * Default implementation of udelay() didn't work correctly when tick was
 * shorter than one millisecond.
 */
// 修改过
void udelay (uint32_t us)
{
	osal_timert qtime;
	osal_timer_start(&qtime, us);

	while(!osal_timer_is_expired(&qtime));
}

// 修改过
int gettimeofday(timeval *tp, void *tzp)
{
    /* zxr: 日志和 DC 初始化使用毫秒级系统时间，实时超时独立使用 TIM2。 */
    uint32_t ms = HAL_GetTick();
    tp->tv_sec = ms / 1000U;
    tp->tv_usec = (ms % 1000U) * 1000U;
    return 0;
}

int osal_usleep (uint32 usec)
{
   udelay(usec);
   return 0;
}

int osal_gettimeofday(timeval *tv, timezone *tz)
{
   return gettimeofday(tv, tz);
}

ec_timet osal_current_time (void)
{
   timeval current_time;
   ec_timet return_value;

   gettimeofday (&current_time, 0);
   return_value.sec = current_time.tv_sec;
   return_value.usec = current_time.tv_usec;
   return return_value;
}

void osal_timer_start(osal_timert *self, uint32 timeout_usec)
{
    /* zxr: 只记录起始计数和等待时长，不拼接秒数。 */
    self->start_us = get_us();
    self->timeout_us = timeout_usec;
}

boolean osal_timer_is_expired(osal_timert *self)
{
    /* zxr: 无符号减法自动处理回绕；从起点到检查时刻须小于约 71.6 分钟。 */
    return (uint32_t)(get_us() - self->start_us) >= self->timeout_us;
}

void *osal_malloc(size_t size)
{
   return malloc(size);
}

void osal_free(void *ptr)
{
   free(ptr);
}

// 注释掉
int osal_thread_create(void *thandle, int stacksize, void *func, void *param)
{
   // thandle = task_spawn ("worker", func, 6,stacksize, param);
   // if(!thandle)
   // {
   //    return 0;
   // }
   return 1;
}

// 注释掉
int osal_thread_create_rt(void *thandle, int stacksize, void *func, void *param)
{
   // thandle = task_spawn ("worker_rt", func, 15 ,stacksize, param);
   // if(!thandle)
   // {
   //    return 0;
   // }
   return 1;
}
