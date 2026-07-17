#ifndef __RM_REFEREE_SYSTEM_FULL_HPP__
#define __RM_REFEREE_SYSTEM_FULL_HPP__

/**
 * RoboMaster 2026 裁判系统串口协议 V1.2.0（20260209）
 * 完整版定义文件。
 *
 * 包含：
 * - 常规链路 / 图传链路的结构体定义
 * - 帧缓存结构体（RMRefereeSystemData_t）
 * - 所有命令码对应的全局变量声明
 * - HAL 串口相关宏定义
 * - RMRefereeSystemParse() 解析入口函数声明
 *
 * 若官方示例结构体命名与字段含义不直观，会做语义化重命名，
 * 并在注释中保留"原始名称"说明。
 */

#include "main.h"
#include "RMRefereeSystemCRC.hpp"

#pragma pack(push, 1)

/*======================================================================*/
/*                          HAL 串口相关宏定义                           */
/*======================================================================*/

#define HuartHandle_RMRefereeSystem   huart6          // 裁判系统所用 UART 句柄
#define HuartDistance_RMRefereeSystem  USART6          // 对应 USART 外设

/*======================================================================*/
/*                          公共帧格式                                  */
/*======================================================================*/

/**
 * 帧头结构体（5 字节）
 */
typedef struct
{
    uint8_t  SOF;          // 起始字节，固定 0xA5
    uint16_t data_length;  // 数据段长度（不含帧头 / cmd_id / CRC16）
    uint8_t  seq;          // 包序号
    uint8_t  CRC8;         // 帧头 CRC8
} frame_header_t;

/**
 * 所有命令码 ID 枚举（串口协议表 1-4）
 */
typedef enum
{
    /* ==================== 常规链路 ==================== */
    RM_CMD_GAME_STATUS              = 0x0001,  // 比赛状态数据
    RM_CMD_GAME_RESULT              = 0x0002,  // 比赛结果数据
    RM_CMD_GAME_ROBOT_HP            = 0x0003,  // 机器人血量数据

    RM_CMD_EVENT_DATA               = 0x0101,  // 场地事件数据
    RM_CMD_REFEREE_WARNING          = 0x0104,  // 裁判警告数据
    RM_CMD_DART_INFO                = 0x0105,  // 飞镖发射相关数据

    RM_CMD_ROBOT_STATUS             = 0x0201,  // 机器人性能体系数据
    RM_CMD_POWER_HEAT_DATA          = 0x0202,  // 实时底盘缓冲能量和枪口热量
    RM_CMD_ROBOT_POSITION           = 0x0203,  // 机器人位置数据
    RM_CMD_ROBOT_BUFF               = 0x0204,  // 机器人增益数据
    RM_CMD_ROBOT_HURT               = 0x0206,  // 伤害状态数据
    RM_CMD_SHOOT_DATA               = 0x0207,  // 实时射击数据
    RM_CMD_PROJECTILE_ALLOWANCE     = 0x0208,  // 允许发弹量
    RM_CMD_RFID_STATUS              = 0x0209,  // RFID 模块状态
    RM_CMD_DART_CLIENT_CMD          = 0x020A,  // 飞镖选手端指令数据
    RM_CMD_GROUND_ROBOT_POSITION    = 0x020B,  // 地面机器人位置数据
    RM_CMD_SENTRY_MARK_INFO         = 0x020C,  // 雷达标记进度数据
    RM_CMD_SENTRY_INFO              = 0x020D,  // 哨兵自主决策信息同步
    RM_CMD_RADAR_INFO               = 0x020E,  // 雷达自主决策信息同步

    /* ==================== 图传链路（机器人交互） ==================== */
    RM_CMD_ROBOT_INTERACTION        = 0x0301,  // 机器人交互数据
    RM_CMD_CUSTOM_CONTROLLER_TO_ROBOT = 0x0302,  // 自定义控制器与机器人交互数据
    RM_CMD_MAP_COMMAND              = 0x0303,  // 选手端小地图交互数据
    // 0x0304 已删除
    RM_CMD_MAP_ROBOT_DATA           = 0x0305,  // 选手端接收雷达数据
    RM_CMD_CONTROLLER_TO_CLIENT     = 0x0306,  // 自定义控制器与选手端交互数据
    RM_CMD_MAP_PATH_DATA            = 0x0307,  // 选手端小地图接收路径数据
    RM_CMD_MAP_CUSTOM_INFO          = 0x0308,  // 选手端小地图自定义消息
    RM_CMD_ROBOT_TO_CONTROLLER      = 0x0309,  // 机器人发送给自定义控制器的数据
    RM_CMD_ROBOT_TO_CUSTOM_CLIENT   = 0x0310,  // 机器人发送给自定义客户端的数据
    RM_CMD_CUSTOM_CLIENT_TO_ROBOT   = 0x0311,  // 自定义客户端发送给机器人的指令
} rm_cmd_id_t;

/**
 * 通用帧结构体（用于拆帧后的完整信息）
 */
typedef struct
{
    frame_header_t    header;
    uint16_t          cmd_id;
    const uint8_t*    data;      // 指向数据段的指针（不做具体类型映射）
    uint16_t          data_len;  // 数据段长度
    uint16_t          crc16;     // 帧尾 CRC16（已从原始帧解析出）
} frame_t;


/*======================================================================*/
/*                 0x0001 比赛状态数据（1Hz 频率发送）                    */
/*======================================================================*/

typedef struct
{
    uint8_t  game_type     : 4;  // 比赛类型（1:超级对抗赛 2:单项赛 3:人工智能挑战赛 4:联盟赛3V3 5:联盟赛1V1）
    uint8_t  game_progress : 4;  // 当前比赛阶段（0:未开始 1:准备 2:自检 3:5s倒计时 4:对战中 5:比赛结算中）
    uint16_t stage_remain_time;  // 当前阶段剩余时间（秒）
    uint64_t SyncTimeStamp;      // UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;


/*======================================================================*/
/*                 0x0002 比赛结果数据                                   */
/*======================================================================*/

typedef struct
{
    uint8_t winner;  // 0 平局；1 红方胜利；2 蓝方胜利
} game_result_t;


/*======================================================================*/
/*             0x0003 机器人血量数据（1Hz 频率发送）                      */
/*======================================================================*/

typedef struct
{
    uint16_t ally_1_robot_HP;   // 1 号英雄机器人血量，未上场以及罚下为 0
    uint16_t ally_2_robot_HP;   // 2 号工程机器人血量
    uint16_t ally_3_robot_HP;   // 3 号步兵机器人血量
    uint16_t ally_4_robot_HP;   // 4 号步兵机器人血量
    uint16_t reserved;          // 保留位
    uint16_t ally_7_robot_HP;   // 7 号哨兵机器人血量
    uint16_t ally_outpost_HP;   // 前哨站血量
    uint16_t ally_base_HP;      // 基地血量
} game_robot_HP_t;


/*======================================================================*/
/*            0x0101 场地事件数据（1Hz 频率发送）                         */
/*======================================================================*/

/**
 * 原始官方示例结构体中 event_data 为 uint32_t 整数。
 * 这里按协议表 1-8 的 bit 分布拆成有语义的位段：
 *
 * 原字段名：event_data（uint32_t）
 * 修正后拆分为：
 *   - ally_supply_no_resource_occupied (bit0)
 *   - ally_supply_with_resource_occupied (bit1)
 *   - ally_supply_rmul_occupied (bit2)
 *   - ally_small_power_rune_status (bit3-4)
 *   - ally_big_power_rune_status (bit5-6)
 *   - ally_central_highland_status (bit7-8)
 *   - ally_trapezoidal_highland_status (bit9-10)
 *   - enemy_dart_last_hit_time (bit11-19)
 *   - enemy_dart_last_hit_target (bit20-22)
 *   - center_buff_point_status (bit23-24)
 *   - ally_fortress_buff_status (bit25-26)
 *   - ally_outpost_buff_status (bit27-28)
 *   - ally_base_buff_occupied (bit29)
 *   - reserved (bit30-31)
 */
typedef struct
{
    uint32_t ally_supply_no_resource_occupied   : 1;  // bit 0：己方与资源区不重叠的补给区占领状态
    uint32_t ally_supply_with_resource_occupied : 1;  // bit 1：己方与资源区重叠的补给区占领状态
    uint32_t ally_supply_rmul_occupied          : 1;  // bit 2：己方补给区的占领状态（仅 RMUL 适用）
    uint32_t ally_small_power_rune_status       : 2;  // bit 3-4：己方小能量机关激活状态（0:未激活 1:已激活 2:正在激活）
    uint32_t ally_big_power_rune_status         : 2;  // bit 5-6：己方大能量机关激活状态（0:未激活 1:已激活 2:正在激活）
    uint32_t ally_central_highland_status       : 2;  // bit 7-8：己方中央高地占领状态（1:己方占领 2:对方占领）
    uint32_t ally_trapezoidal_highland_status   : 2;  // bit 9-10：己方梯形高地占领状态（1:已占领）
    uint32_t enemy_dart_last_hit_time           : 9;  // bit 11-19：对方飞镖最后一次击中己方前哨站/基地的时间（0-420）
    uint32_t enemy_dart_last_hit_target         : 3;  // bit 20-22：对方飞镖最后一次击中的具体目标（0:默认 1:前哨站 2:基地固定 3:基地随机固定 4:基地随机移动 5:基地末端移动）
    uint32_t center_buff_point_status           : 2;  // bit 23-24：中心增益点占领状态（0:未占 1:己方 2:对方 3:双方，仅 RMUL）
    uint32_t ally_fortress_buff_status          : 2;  // bit 25-26：己方堡垒增益点占领状态（0:未占 1:己方 2:对方 3:双方）
    uint32_t ally_outpost_buff_status           : 2;  // bit 27-28：己方前哨站增益点占领状态（0:未占 1:己方 2:对方）
    uint32_t ally_base_buff_occupied            : 1;  // bit 29：己方基地增益点占领状态（1:已占领）
    uint32_t reserved                           : 2;  // bit 30-31：保留位
} event_data_t;                                       // 原始名称：event_data_t（原为 uint32_t event_data）


/*======================================================================*/
/*                 0x0104 裁判警告数据                                   */
/*======================================================================*/

typedef struct
{
    uint8_t level;              // 最后一次受到判罚的等级
    uint8_t offending_robot_id; // 违规机器人 ID
    uint8_t count;              // 对应判罚等级的违规次数
} referee_warning_t;


/*======================================================================*/
/*            0x0105 飞镖发射相关数据（1Hz 频率发送）                     */
/*======================================================================*/

/**
 * 原始官方示例结构体 dart_info_t 中 dart_info 为 uint16_t 整数。
 * 这里按协议表 1-10 的 bit 分布拆成有语义的位段：
 *
 * 原字段名：dart_info（uint16_t）
 * 修正后拆分为：
 *   - last_hit_target (bit0-2)：最近一次己方飞镖命中的目标
 *       0:开局/未命中 1:前哨站 2:基地固定目标 3:基地随机固定目标
 *       4:基地随机移动目标 5:基地末端移动目标
 *   - hit_count (bit3-5)：对方最近被击中目标累计被击中次数（0-4）
 *   - selected_target (bit6-8)：飞镖当前选定的击打目标
 *       0:未选定/选定前哨站 1:基地固定目标 2:基地随机固定目标
 *       3:基地随机移动目标 4:基地末端移动目标
 *   - reserved (bit9-15)：保留
 */
typedef struct
{
    uint8_t  dart_remaining_time;     // 剩余时间（秒）
    uint16_t last_hit_target : 3;     // bit0-2：最近一次命中的目标
    uint16_t hit_count       : 3;     // bit3-5：对方最近被击中的目标累计次数
    uint16_t selected_target : 3;     // bit6-8：当前选定的击打目标
    uint16_t reserved        : 7;     // bit9-15：保留
} dart_info_t;                        // 原始名称：dart_info_t（原为 uint16_t dart_info）


/*======================================================================*/
/*          0x0201 机器人性能体系数据（10Hz 频率发送）                    */
/*======================================================================*/

typedef struct
{
    uint8_t  robot_id;                           // 机器人 ID
    uint8_t  robot_level;                        // 机器人等级（1/2/3）
    uint16_t current_HP;                         // 当前血量
    uint16_t maximum_HP;                         // 血量上限
    uint16_t shooter_barrel_cooling_value;       // 机器人枪口每秒冷却值
    uint16_t shooter_barrel_heat_limit;          // 机器人枪口热量上限
    uint16_t chassis_power_limit;                // 机器人底盘功率上限
    uint8_t  power_management_gimbal_output  : 1;  // 主控电源输出：云台口
    uint8_t  power_management_chassis_output : 1;  // 主控电源输出：底盘口
    uint8_t  power_management_shooter_output : 1;  // 主控电源输出：射击口
} robot_status_t;


/*======================================================================*/
/*        0x0202 实时底盘缓冲能量和枪口热量（以 50Hz 频率发送）           */
/*======================================================================*/

/**
 * V1.2.0 协议中，前两个 uint16 和一个 float 为保留位。
 * 原始示例字段名为 chassis_voltage / chassis_current / chassis_power，
 * 此处按照文档修订为 reserved1/2/3。
 */
typedef struct
{
    uint16_t reserved1;                 // 保留位（原名：chassis_voltage）
    uint16_t reserved2;                 // 保留位（原名：chassis_current）
    float    reserved3;                 // 保留位（原名：chassis_power）
    uint16_t buffer_energy;             // 底盘功率缓冲区剩余能量（J）
    uint16_t shooter_17mm_barrel_heat;  // 17mm 枪口热量
    uint16_t shooter_42mm_barrel_heat;  // 42mm 枪口热量
} power_heat_data_t;


/*======================================================================*/
/*            0x0203 机器人位置数据（10Hz 频率发送）                      */
/*======================================================================*/

typedef struct
{
    float x;      // 位置 x 坐标（m）
    float y;      // 位置 y 坐标（m）
    float angle;  // 测速模块朝向（度），正北为 0°
} robot_pos_t;


/*======================================================================*/
/*        0x0204 机器人增益和能量数据（1Hz 频率发送）                     */
/*======================================================================*/

/**
 * 原始官方示例结构体中最后一个字节 remaining_energy 为 uint8_t 整数。
 * 这里按协议的 bit 分布拆成有语义的位段，每个 bit 为 1 表示剩余能量 ≥ 对应百分比：
 *
 * 原字段名：remaining_energy（uint8_t）
 * 修正后拆分为：
 *   - energy_ge_125_pct (bit0)：剩余能量 ≥ 125%
 *   - energy_ge_100_pct (bit1)：剩余能量 ≥ 100%
 *   - energy_ge_50_pct  (bit2)：剩余能量 ≥ 50%
 *   - energy_ge_30_pct  (bit3)：剩余能量 ≥ 30%
 *   - energy_ge_15_pct  (bit4)：剩余能量 ≥ 15%
 *   - energy_ge_5_pct   (bit5)：剩余能量 ≥ 5%
 *   - energy_ge_1_pct   (bit6)：剩余能量 ≥ 1%
 *   - energy_reserved   (bit7)：保留
 */
typedef struct
{
    uint8_t  recovery_buff;            // 机器人回血增益百分比（值为10表示每秒恢复血量上限的10%）
    uint16_t cooling_buff;             // 射击热量冷却增益具体值（直接值）
    uint8_t  defence_buff;             // 防御增益百分比
    uint8_t  vulnerability_buff;       // 负防御增益百分比
    uint16_t attack_buff;              // 攻击增益百分比
    uint8_t  energy_ge_125_pct : 1;    // bit 0：剩余能量 ≥ 125%
    uint8_t  energy_ge_100_pct : 1;    // bit 1：剩余能量 ≥ 100%
    uint8_t  energy_ge_50_pct  : 1;    // bit 2：剩余能量 ≥ 50%
    uint8_t  energy_ge_30_pct  : 1;    // bit 3：剩余能量 ≥ 30%
    uint8_t  energy_ge_15_pct  : 1;    // bit 4：剩余能量 ≥ 15%
    uint8_t  energy_ge_5_pct   : 1;    // bit 5：剩余能量 ≥ 5%
    uint8_t  energy_ge_1_pct   : 1;    // bit 6：剩余能量 ≥ 1%
    uint8_t  energy_reserved   : 1;    // bit 7：保留
} buff_t;                                              // 原始名称：buff_t（原为 uint8_t remaining_energy）


/*======================================================================*/
/*                 0x0206 伤害状态数据                                   */
/*======================================================================*/

/**
 * 原始字段为 uint8_t 单字节，按 bit 拆分为：
 *   - armor_id (bit0-3)：当扣血原因为装甲伤害时，代表装甲 ID
 *   - HP_deduction_reason (bit4-7)：血量变化类型
 *       0:装甲伤害 1:模块掉线 2:超射速 3:超热量 4:超底盘功率 5:装甲撞击
 */
typedef struct
{
    uint8_t armor_id            : 4;  // bit0-3：装甲 / 测速模块 ID
    uint8_t HP_deduction_reason : 4;  // bit4-7：扣血原因
} hurt_data_t;


/*======================================================================*/
/*            0x0207 实时射击数据                                        */
/*======================================================================*/

typedef struct
{
    uint8_t bullet_type;         // 弹丸类型（1: 17mm 弹丸；2: 42mm 弹丸）
    uint8_t shooter_number;      // 发射机构 ID（1: 17mm 第1个 2: 17mm 第2个 3: 42mm）
    uint8_t launching_frequency; // 弹丸射速（Hz）
    float   initial_speed;       // 弹丸初速度（m/s）
} shoot_data_t;


/*======================================================================*/
/*            0x0208 允许发弹量（10Hz 频率发送）                          */
/*======================================================================*/

typedef struct
{
    uint16_t projectile_allowance_17mm;      // 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;      // 42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;            // 剩余金币数量
    uint16_t projectile_allowance_fortress;  // 堡垒允许发弹量
} projectile_allowance_t;


/*======================================================================*/
/*            0x0209 RFID 模块状态（3Hz 频率发送）                       */
/*======================================================================*/

/**
 * 原始官方示例结构体中 rfid_status 为 uint32_t、rfid_status_2 为 uint8_t 整数。
 * 这里按协议表 1-18 的 bit 分布拆成有语义的位段，每个 bit 值为 1 表示已检测到该增益点 RFID 卡。
 *
 * 原字段名：rfid_status（uint32_t）+ rfid_status_2（uint8_t）
 * 修正后拆分为：
 *   - ally_base_buff (bit0) ~ ally_tunnel_trapezoid_high (bit31)：见下方各字段注释
 *   - enemy_tunnel_highway_below (bit0) ~ enemy_tunnel_trapezoid_high (bit5)：见下方各字段注释
 *   - rfid_reserved (bit6-7)：保留
 */
typedef struct
{
    /* ---- 第一段 uint32_t（32 bit）（原 rfid_status） ---- */
    uint32_t ally_base_buff                     : 1;  // bit 0：己方基地增益点
    uint32_t ally_central_highland_buff         : 1;  // bit 1：己方中央高地增益点
    uint32_t enemy_central_highland_buff        : 1;  // bit 2：对方中央高地增益点
    uint32_t ally_trapezoidal_highland_buff     : 1;  // bit 3：己方梯形高地增益点
    uint32_t enemy_trapezoidal_highland_buff    : 1;  // bit 4：对方梯形高地增益点
    uint32_t ally_ramp_before_ally_side         : 1;  // bit 5：己方飞坡前（靠近己方一侧）
    uint32_t ally_ramp_after_ally_side          : 1;  // bit 6：己方飞坡后（靠近己方一侧）
    uint32_t enemy_ramp_before_enemy_side       : 1;  // bit 7：对方飞坡前（靠近对方一侧）
    uint32_t enemy_ramp_after_enemy_side        : 1;  // bit 8：对方飞坡后（靠近对方一侧）
    uint32_t ally_terrain_central_highland_below : 1; // bit 9：己方地形跨越（中央高地下方）
    uint32_t ally_terrain_central_highland_above : 1; // bit 10：己方地形跨越（中央高地上方）
    uint32_t enemy_terrain_central_highland_below: 1; // bit 11：对方地形跨越（中央高地下方）
    uint32_t enemy_terrain_central_highland_above: 1; // bit 12：对方地形跨越（中央高地上方）
    uint32_t ally_terrain_highway_below         : 1;  // bit 13：己方地形跨越（公路下方）
    uint32_t ally_terrain_highway_above         : 1;  // bit 14：己方地形跨越（公路上方）
    uint32_t enemy_terrain_highway_below        : 1;  // bit 15：对方地形跨越（公路下方）
    uint32_t enemy_terrain_highway_above        : 1;  // bit 16：对方地形跨越（公路上方）
    uint32_t ally_fortress_buff                 : 1;  // bit 17：己方堡垒增益点
    uint32_t ally_outpost_buff                  : 1;  // bit 18：己方前哨站增益点
    uint32_t ally_supply_no_resource            : 1;  // bit 19：己方与资源区不重叠的补给区/RMUL补给区
    uint32_t ally_supply_with_resource          : 1;  // bit 20：己方与资源区重叠的补给区
    uint32_t ally_assembly_buff                 : 1;  // bit 21：己方装配增益点
    uint32_t enemy_assembly_buff                : 1;  // bit 22：对方装配增益点
    uint32_t center_buff_rmul                   : 1;  // bit 23：中心增益点（仅 RMUL 适用）
    uint32_t enemy_fortress_buff                : 1;  // bit 24：对方堡垒增益点
    uint32_t enemy_outpost_buff                 : 1;  // bit 25：对方前哨站增益点
    uint32_t ally_tunnel_highway_below          : 1;  // bit 26：己方隧道（靠近己方公路区下方）
    uint32_t ally_tunnel_highway_mid            : 1;  // bit 27：己方隧道（靠近己方公路区中间）
    uint32_t ally_tunnel_highway_above          : 1;  // bit 28：己方隧道（靠近己方公路区上方）
    uint32_t ally_tunnel_trapezoid_low          : 1;  // bit 29：己方隧道（靠近己方梯形高地较低处）
    uint32_t ally_tunnel_trapezoid_mid          : 1;  // bit 30：己方隧道（靠近己方梯形高地较中间）
    uint32_t ally_tunnel_trapezoid_high         : 1;  // bit 31：己方隧道（靠近己方梯形高地较高处）

    /* ---- 第二段 uint8_t（6 bit + 2 保留）（原 rfid_status_2） ---- */
    uint8_t  enemy_tunnel_highway_below         : 1;  // bit 0：对方隧道（靠近对方公路一侧下方）
    uint8_t  enemy_tunnel_highway_mid           : 1;  // bit 1：对方隧道（靠近对方公路一侧中间）
    uint8_t  enemy_tunnel_highway_above         : 1;  // bit 2：对方隧道（靠近对方公路一侧上方）
    uint8_t  enemy_tunnel_trapezoid_low         : 1;  // bit 3：对方隧道（靠近对方梯形高地较低处）
    uint8_t  enemy_tunnel_trapezoid_mid         : 1;  // bit 4：对方隧道（靠近对方梯形高地较中间）
    uint8_t  enemy_tunnel_trapezoid_high        : 1;  // bit 5：对方隧道（靠近对方梯形高地较高处）
    uint8_t  rfid_reserved                      : 2;  // bit 6-7：保留
} rfid_status_t;                                      // 原始名称：rfid_status_t（原为 uint32_t rfid_status + uint8_t rfid_status_2）


/*======================================================================*/
/*            0x020A 飞镖选手端指令数据（10Hz 频率发送）                  */
/*======================================================================*/

typedef struct
{
    uint8_t  dart_launch_opening_status;  // 闸门状态（1:关闭 2:正在开关 0:已开启）
    uint8_t  reserved;                    // 保留位
    uint16_t target_change_time;          // 切换击打目标时比赛剩余时间
    uint16_t latest_launch_cmd_time;      // 最后一次确认发射指令时比赛剩余时间
} dart_client_cmd_t;


/*======================================================================*/
/*            0x020B 地面机器人位置数据（1Hz 频率发送）                   */
/*======================================================================*/

typedef struct
{
    float hero_x;        // 己方英雄机器人 x 坐标（m）
    float hero_y;        // 己方英雄机器人 y 坐标（m）
    float engineer_x;    // 己方工程机器人 x 坐标（m）
    float engineer_y;    // 己方工程机器人 y 坐标（m）
    float standard_3_x;  // 己方 3 号步兵 x 坐标（m）
    float standard_3_y;  // 己方 3 号步兵 y 坐标（m）
    float standard_4_x;  // 己方 4 号步兵 x 坐标（m）
    float standard_4_y;  // 己方 4 号步兵 y 坐标（m）
    float reserved1;     // 保留位
    float reserved2;     // 保留位
} ground_robot_position_t;


/*======================================================================*/
/*            0x020C 雷达标记进度数据（1Hz 频率发送）                     */
/*======================================================================*/

/**
 * 原始官方示例结构体中 mark_progress 为 uint16_t 整数。
 * 这里按协议表 1-21 的 bit 分布拆成有语义的位段：
 *
 * 原字段名：mark_progress（uint16_t）
 * 修正后拆分为：
 *   - enemy_hero_vulnerable (bit0)：对方英雄易伤
 *   - enemy_engineer_vulnerable (bit1)：对方工程易伤
 *   - enemy_infantry_3_vulnerable (bit2)：对方3号步兵易伤
 *   - enemy_infantry_4_vulnerable (bit3)：对方4号步兵易伤
 *   - enemy_aerial_special_mark (bit4)：对方空中机器人特殊标识
 *   - enemy_sentry_vulnerable (bit5)：对方哨兵易伤
 *   - ally_hero_special_mark (bit6) ~ ally_sentry_special_mark (bit11)：己方各机器人特殊标识
 *   - reserved (bit12-15)：保留
 */
typedef struct
{
    uint16_t enemy_hero_vulnerable          : 1;  // bit 0：对方 1 号英雄机器人易伤情况
    uint16_t enemy_engineer_vulnerable      : 1;  // bit 1：对方 2 号工程机器人易伤情况
    uint16_t enemy_infantry_3_vulnerable    : 1;  // bit 2：对方 3 号步兵机器人易伤情况
    uint16_t enemy_infantry_4_vulnerable    : 1;  // bit 3：对方 4 号步兵机器人易伤情况
    uint16_t enemy_aerial_special_mark      : 1;  // bit 4：对方空中机器人特殊标识情况
    uint16_t enemy_sentry_vulnerable        : 1;  // bit 5：对方哨兵机器人易伤情况
    uint16_t ally_hero_special_mark         : 1;  // bit 6：己方 1 号英雄机器人特殊标识情况
    uint16_t ally_engineer_special_mark     : 1;  // bit 7：己方 2 号工程机器人特殊标识情况
    uint16_t ally_infantry_3_special_mark   : 1;  // bit 8：己方 3 号步兵机器人特殊标识情况
    uint16_t ally_infantry_4_special_mark   : 1;  // bit 9：己方 4 号步兵机器人特殊标识情况
    uint16_t ally_aerial_special_mark       : 1;  // bit 10：己方空中机器人特殊标识情况
    uint16_t ally_sentry_special_mark       : 1;  // bit 11：己方哨兵机器人特殊标识情况
    uint16_t reserved                       : 4;  // bit 12-15：保留位
} radar_mark_data_t;                              // 原始名称：radar_mark_data_t（原为 uint16_t mark_progress）


/*======================================================================*/
/*            0x020D 哨兵自主决策信息同步（1Hz 频率发送）                 */
/*======================================================================*/

/**
 * 原始官方示例结构体中 sentry_info 为 uint32_t、sentry_info_2 为 uint16_t 整数。
 * 这里按协议表 1-22 的 bit 分布拆成有语义的位段：
 *
 * 原字段名：sentry_info（uint32_t）+ sentry_info_2（uint16_t）
 * 修正后拆分为：
 *   第一段（原 sentry_info）：
 *   - exchanged_projectile_allowance (bit0-10)：哨兵成功兑换的允许发弹量
 *   - remote_exchange_projectile_count (bit11-14)：远程兑换发弹量次数
 *   - remote_exchange_hp_count (bit15-18)：远程兑换血量次数
 *   - can_confirm_free_revive (bit19)：是否可确认免费复活
 *   - can_exchange_instant_revive (bit20)：是否可兑换立即复活
 *   - instant_revive_cost (bit21-30)：立即复活花费金币数
 *   - sentry_reserved_1 (bit31)：保留
 *   第二段（原 sentry_info_2）：
 *   - is_disengaged (bit0)：是否处于脱战状态
 *   - remaining_exchangeable_17mm (bit1-11)：17mm 剩余可兑换发弹量
 *   - sentry_posture (bit12-13)：哨兵姿态（1:进攻 2:防御 3:移动）
 *   - power_rune_activatable (bit14)：能量机关是否可激活
 *   - sentry_reserved_2 (bit15)：保留
 */
typedef struct
{
    /* ---- 第一段 uint32_t（原 sentry_info） ---- */
    uint32_t exchanged_projectile_allowance    : 11; // bit 0-10：除远程兑换外，哨兵成功兑换的允许发弹量
    uint32_t remote_exchange_projectile_count  : 4;  // bit 11-14：哨兵成功远程兑换允许发弹量的次数
    uint32_t remote_exchange_hp_count          : 4;  // bit 15-18：哨兵成功远程兑换血量的次数
    uint32_t can_confirm_free_revive           : 1;  // bit 19：哨兵当前是否可以确认免费复活（1:可以）
    uint32_t can_exchange_instant_revive       : 1;  // bit 20：哨兵当前是否可以兑换立即复活（1:可以）
    uint32_t instant_revive_cost               : 10; // bit 21-30：兑换立即复活需要花费的金币数
    uint32_t sentry_reserved_1                 : 1;  // bit 31：保留

    /* ---- 第二段 uint16_t（原 sentry_info_2） ---- */
    uint16_t is_disengaged                     : 1;  // bit 0：哨兵当前是否处于脱战状态（1:脱战）
    uint16_t remaining_exchangeable_17mm       : 11; // bit 1-11：队伍 17mm 允许发弹量的剩余可兑换数
    uint16_t sentry_posture                    : 2;  // bit 12-13：哨兵当前姿态（1:进攻 2:防御 3:移动）
    uint16_t power_rune_activatable            : 1;  // bit 14：己方能量机关是否可进入激活状态（1:可激活）
    uint16_t sentry_reserved_2                 : 1;  // bit 15：保留位
} sentry_info_t;                                     // 原始名称：sentry_info_t（原为 uint32_t sentry_info + uint16_t sentry_info_2）


/*======================================================================*/
/*            0x020E 雷达自主决策信息同步（1Hz 频率发送）                 */
/*======================================================================*/

/**
 * 原始官方示例结构体中 radar_info 为 uint8_t 整数。
 * 这里按协议表 1-23 的 bit 分布拆成有语义的位段：
 *
 * 原字段名：radar_info（uint8_t）
 * 修正后拆分为：
 *   - double_vulnerability_chances (bit0-1)：雷达拥有触发双倍易伤的机会数（0-2）
 *   - enemy_double_vulnerability_on (bit2)：对方是否正在被触发双倍易伤
 *   - ally_encryption_level (bit3-4)：己方加密等级（开局为1，最高3）
 *   - can_modify_key (bit5)：当前是否可以修改密钥
 *   - reserved (bit6-7)：保留
 */
typedef struct
{
    uint8_t double_vulnerability_chances    : 2;  // bit 0-1：雷达拥有触发双倍易伤的机会数（0-2）
    uint8_t enemy_double_vulnerability_on   : 1;  // bit 2：对方是否正在被触发双倍易伤（1:是）
    uint8_t ally_encryption_level           : 2;  // bit 3-4：己方加密等级（即对方干扰波难度等级，开局为1，最高3）
    uint8_t can_modify_key                  : 1;  // bit 5：当前是否可以修改密钥（1:可修改）
    uint8_t reserved                        : 2;  // bit 6-7：保留位
} radar_info_t;                                   // 原始名称：radar_info_t（原为 uint8_t radar_info）


/*======================================================================*/
/*            0x0301 机器人交互数据                                      */
/*======================================================================*/

typedef struct
{
    uint16_t data_cmd_id;   // 子内容 ID
    uint16_t sender_id;     // 发送者 ID
    uint16_t receiver_id;   // 接收者 ID
    // 后续为内容数据段，长度不超过 112 字节
} robot_interaction_header_t;

/* 选手端图形绘制相关结构（子内容 ID 0x0101/0x0102/0x0103/0x0104/0x0110） */

typedef struct
{
    uint8_t  figure_name[3];
    uint32_t operate_type : 3;
    uint32_t figure_type  : 3;
    uint32_t layer        : 4;
    uint32_t color        : 4;
    uint32_t details_a    : 9;
    uint32_t details_b    : 9;
    uint32_t width        : 10;
    uint32_t start_x      : 11;
    uint32_t start_y      : 11;
    uint32_t details_c    : 10;
    uint32_t details_d    : 11;
    uint32_t details_e    : 11;
} interaction_figure_t;

typedef struct
{
    interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

typedef struct
{
    interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

typedef struct
{
    interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;


/*======================================================================*/
/*       0x0302 自定义控制器与机器人交互数据                              */
/*======================================================================*/

typedef struct
{
    uint8_t data[30];  // 实际长度由协议约束（最大 30 字节）
} custom_robot_data_t;


/*======================================================================*/
/*            0x0303 选手端小地图交互数据                                 */
/*======================================================================*/

typedef struct
{
    float    target_position_x;  // 目标位置 x（m）
    float    target_position_y;  // 目标位置 y（m）
    uint8_t  cmd_keyboard;       // 键盘按键通用键值
    uint8_t  target_robot_id;    // 对方机器人 ID
    uint16_t cmd_source;         // 信息来源 ID
} map_command_t;


/*======================================================================*/
/*            0x0305 选手端接收雷达数据                                   */
/*======================================================================*/

typedef struct
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;


/*======================================================================*/
/*            0x0307 选手端小地图接收路径数据                              */
/*======================================================================*/

typedef struct
{
    uint8_t  intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t   delta_x[49];
    int8_t   delta_y[49];
    uint16_t sender_id;
} map_path_data_t;


/*======================================================================*/
/*            0x0308 选手端小地图自定义消息                                */
/*======================================================================*/

typedef struct
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t  user_data[30];  // UTF-16 编码，注意大小端
} custom_info_t;


/*======================================================================*/
/*            0x0309 机器人发送给自定义控制器的数据                        */
/*======================================================================*/

typedef struct
{
    uint8_t data[30];
} robot_custom_data_t;


/*======================================================================*/
/*            0x0310 机器人发送给自定义客户端的数据                        */
/*======================================================================*/

typedef struct
{
    uint8_t data[300];
} robot_custom_data_2_t;


/*======================================================================*/
/*            0x0311 自定义客户端发送给机器人的指令                        */
/*======================================================================*/

typedef struct
{
    uint8_t data[30];
} robot_custom_data_3_t;


#pragma pack(pop)


/*======================================================================*/
/*                   帧缓存结构体与全局变量声明                          */
/*======================================================================*/

/**
 * 帧缓存结构体，用于存储当前正在解析的一帧裁判系统数据
 */
typedef __packed struct
{
    uint8_t  SOF;              // 数据帧起始字节，固定值为 0xA5
    uint16_t data_length;      // 数据帧中 data 的长度
    uint8_t  seq;              // 包序号
    uint8_t  CRC8;             // 帧头 CRC8 校验
    uint16_t cmd_id;           // 命令码 ID
    uint8_t  data[6 + 105];    // 数据区（覆盖大部分常用 cmd 的数据长度）
    uint16_t frame_tail;       // CRC16，整包校验
} RMRefereeSystemData_t;

extern RMRefereeSystemData_t     RMRefereeSystemData;           // 裁判系统当前帧数据

extern unsigned char             MyRefereeSys8Data;              // 裁判系统串口数据接收缓冲区（单字节）

/* ===================== 常规链路命令码对应的全局变量 ===================== */

extern game_status_t             game_status_0x0001;             // 0x0001 比赛状态数据
extern game_result_t             game_result_0x0002;             // 0x0002 比赛结果数据
extern game_robot_HP_t           game_robot_HP_0x0003;           // 0x0003 机器人血量数据

extern event_data_t              event_data_0x0101;              // 0x0101 场地事件数据
extern referee_warning_t         referee_warning_0x0104;         // 0x0104 裁判警告数据
extern dart_info_t               dart_info_0x0105;               // 0x0105 飞镖发射相关数据

extern robot_status_t            robot_status_0x0201;            // 0x0201 机器人性能体系数据
extern power_heat_data_t         power_heat_data_0x0202;         // 0x0202 实时底盘缓冲能量和枪口热量
extern robot_pos_t               robot_pos_0x0203;               // 0x0203 机器人位置数据
extern buff_t                    buff_0x0204;                    // 0x0204 机器人增益和能量数据
extern hurt_data_t               hurt_data_0x0206;               // 0x0206 伤害状态数据
extern shoot_data_t              shoot_data_0x0207;              // 0x0207 实时射击数据
extern projectile_allowance_t    projectile_allowance_0x0208;    // 0x0208 允许发弹量
extern rfid_status_t             rfid_status_0x0209;             // 0x0209 RFID 模块状态
extern dart_client_cmd_t         dart_client_cmd_0x020A;         // 0x020A 飞镖选手端指令数据
extern ground_robot_position_t   ground_robot_position_0x020B;   // 0x020B 地面机器人位置数据
extern radar_mark_data_t         radar_mark_data_0x020C;         // 0x020C 雷达标记进度数据
extern sentry_info_t             sentry_info_0x020D;             // 0x020D 哨兵自主决策信息同步
extern radar_info_t              radar_info_0x020E;              // 0x020E 雷达自主决策信息同步


/*======================================================================*/
/*                          函数声明                                    */
/*======================================================================*/

/**
 * @brief  裁判系统解析入口（在串口中断回调中调用）
 */
void RMRefereeSystemParse(void);

#endif // __RM_REFEREE_SYSTEM_FULL_HPP__
