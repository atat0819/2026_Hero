#ifndef LOB_SHOT_HPP
#define LOB_SHOT_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"

#ifdef __cplusplus
extern "C" {
#endif

void lob_shot_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif // LOB_SHOT_HPP
